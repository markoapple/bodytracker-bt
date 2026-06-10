#!/usr/bin/env python3
"""Debug/validation probe for hardwood plank floor patterns.

This intentionally checks for a graph of seams, not just a few repeated lines:
- a primary family of finite supported plank seams;
- a secondary/cross-board family of finite butt joints;
- cross-board joints attached to the primary pattern across multiple lanes.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def clamp01(x):
    return max(0.0, min(1.0, float(x)))


def angle_dist(a, b):
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def norm_angle(a):
    a = a % math.pi
    if a < 0:
        a += math.pi
    return a


def sample_u8(gray, x, y):
    ix = int(round(x)); iy = int(round(y))
    if 0 <= ix < gray.shape[1] and 0 <= iy < gray.shape[0]:
        return float(gray[iy, ix])
    return None


def dark_score(gray, line, samples=8, offset=5.0):
    x1, y1, x2, y2 = line
    dx = x2 - x1; dy = y2 - y1
    length = math.hypot(dx, dy)
    if length < 1:
        return 0.0
    nx = -dy / length; ny = dx / length
    vals = []
    for t in np.linspace(0.12, 0.88, samples):
        x = x1 + t * dx; y = y1 + t * dy
        c = sample_u8(gray, x, y)
        a = sample_u8(gray, x + nx * offset, y + ny * offset)
        b = sample_u8(gray, x - nx * offset, y - ny * offset)
        if c is not None and a is not None and b is not None:
            vals.append(0.5 * (a + b) - c)
    if not vals:
        return 0.0
    return max(0.0, float(np.mean(vals)))


def extract_segments(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    eq = clahe.apply(gray)
    blur = cv2.GaussianBlur(eq, (3, 3), 0)
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_ADV)
    raw = lsd.detect(blur)[0]
    segs = []
    if raw is None:
        return gray, eq, segs, 0
    for item in raw[:, 0, :]:
        x1, y1, x2, y2 = map(float, item)
        dx = x2 - x1; dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < 15.0:
            continue
        angle = norm_angle(math.atan2(dy, dx))
        dark = dark_score(gray, (x1, y1, x2, y2))
        if dark <= 0.6:
            continue
        segs.append({
            "line": (x1, y1, x2, y2),
            "angle": angle,
            "length": length,
            "mid_y": 0.5 * (y1 + y2),
            "dark": dark,
            "score": clamp01(0.18 + 0.50 * min(1.0, length / 120.0) + 0.32 * min(1.0, dark / 28.0)),
        })
    return gray, eq, segs, int(len(raw))


def orientation_peaks(segs):
    hist = np.zeros(180, dtype=np.float32)
    for s in segs:
        deg = int(round(math.degrees(s["angle"]))) % 180
        hist[deg] += s["length"] * (0.2 + min(1.0, s["dark"] / 25.0))
    smooth = np.zeros_like(hist)
    for i in range(180):
        smooth[i] = sum(hist[(i + j) % 180] for j in range(-3, 4))
    order = list(np.argsort(-smooth))
    out = []
    for deg in order:
        if smooth[deg] <= 0:
            break
        theta = math.radians(float(deg))
        if all(angle_dist(theta, existing) >= math.radians(12.0) for existing in out):
            out.append(theta)
        if len(out) >= 8:
            break
    return out

def refine_orientation(seed_theta, segs, image_shape, gray=None, want_short=False):
    best = (float("-inf"), seed_theta, [], [])
    for delta_deg in np.linspace(-5.0, 5.0, 21):
        theta = norm_angle(seed_theta + math.radians(float(delta_deg)))
        clusters = clusters_for(segs, theta, image_shape)
        if want_short:
            usable = [c for c in clusters if 30.0 <= c["total"] <= 270.0 and c["dark"] >= 3.0]
            support = sum(c["score"] for c in usable)
            score = 190.0 * len(usable) + support
            run = usable
        else:
            run = find_primary_run(clusters, image_shape, gray=gray, theta=theta)
            support = sum(c["total"] for c in run)
            score = 280.0 * len(run) + support
        if score > best[0]:
            best = (score, theta, clusters, run)
    return best[1], best[2], best[3]


def merge_intervals(intervals, gap=22.0):
    if not intervals:
        return []
    intervals = sorted((min(a, b), max(a, b)) for a, b in intervals)
    merged = [[intervals[0][0], intervals[0][1]]]
    for a, b in intervals[1:]:
        if a <= merged[-1][1] + gap:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return merged


def clusters_for(segs, theta, image_shape):
    h, w = image_shape[:2]
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    items = []
    for s in segs:
        if angle_dist(s["angle"], theta) > math.radians(10.5):
            continue
        x1, y1, x2, y2 = s["line"]
        mx = 0.5 * (x1 + x2); my = 0.5 * (y1 + y2)
        rho = mx * nx + my * ny
        u1 = x1 * dx + y1 * dy
        u2 = x2 * dx + y2 * dy
        items.append((rho, min(u1, u2), max(u1, u2), s))
    items.sort(key=lambda t: t[0])
    raw_clusters = []
    for rho, u0, u1, s in items:
        if not raw_clusters or abs(rho - raw_clusters[-1]["last_rho"]) > max(14.0, 0.018 * w):
            raw_clusters.append({"rhos": [], "last_rho": rho, "intervals": [], "segments": []})
        c = raw_clusters[-1]
        c["rhos"].append(rho); c["last_rho"] = rho
        c["intervals"].append((u0, u1)); c["segments"].append(s)
    clusters = []
    for c in raw_clusters:
        merged = merge_intervals(c["intervals"])
        total = sum(b - a for a, b in merged)
        longest = max([b - a for a, b in merged] or [0.0])
        dark = float(np.average([s["dark"] for s in c["segments"]], weights=[s["length"] for s in c["segments"]]))
        mid_y = float(np.average([s["mid_y"] for s in c["segments"]], weights=[s["length"] for s in c["segments"]]))
        rho = float(np.average(c["rhos"], weights=[s["length"] for s in c["segments"]]))
        if total >= 38.0 and longest >= 24.0 and dark >= 2.0:
            clusters.append({
                "rho": rho,
                "merged": merged,
                "total": total,
                "longest": longest,
                "dark": dark,
                "mid_y": mid_y,
                "segment_count": len(c["segments"]),
                "score": total * (0.25 + min(1.0, dark / 22.0)),
            })
    clusters.sort(key=lambda c: c["rho"])
    return clusters


def find_primary_run(clusters, image_shape, gray=None, theta=None):
    h, w = image_shape[:2]
    usable = [c for c in clusters if c["total"] >= 50.0 and c["longest"] >= 28.0 and c["dark"] >= 2.0]
    usable.sort(key=lambda c: c["rho"])
    if len(usable) < 4:
        return []
    # Keep the strongest plausible finite seams. This is deliberately not a strict
    # equal-spacing lattice: floorboard images are perspective distorted and often
    # only show fragments. The cross-board graph below is the real confirmation.
    strongest = sorted(usable, key=lambda c: c["score"], reverse=True)[:9]
    strongest.sort(key=lambda c: c["rho"])
    # Reject absurd near-duplicates, but allow uneven perspective spacing.
    filtered = []
    for c in strongest:
        if filtered and abs(c["rho"] - filtered[-1]["rho"]) < 22.0:
            if c["score"] > filtered[-1]["score"]:
                filtered[-1] = c
            continue
        filtered.append(c)
    if len(filtered) < 4:
        return []
    if gray is not None and theta is not None:
        filtered = densify_primary_run(filtered, gray, image_shape, theta)
    total_support = sum(c["total"] for c in filtered)
    lower = sum(1 for c in filtered if c["mid_y"] > 0.34 * h)
    if total_support < 0.62 * w or lower < 2:
        return []
    return filtered


def local_dark_evidence(gray, x, y, theta, offset=4.5):
    nx = -math.sin(theta); ny = math.cos(theta)
    c = sample_u8(gray, x, y)
    a = sample_u8(gray, x + nx * offset, y + ny * offset)
    b = sample_u8(gray, x - nx * offset, y - ny * offset)
    if c is None or a is None or b is None:
        return 0.0
    return max(0.0, 0.5 * (a + b) - c)


def line_scan_intervals(gray, rho, theta, step=4.0, on_thresh=1.9, keep_thresh=1.1, min_run=20.0, gap_allow=14.0):
    h, w = gray.shape[:2]
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    diag = math.hypot(w, h)
    intervals = []
    in_run = False
    run_start = None
    run_end = None
    gap = 0.0
    prev_u = None
    u = -diag
    while u <= diag:
        x = nx * rho + dx * u
        y = ny * rho + dy * u
        inside = 0 <= x < w and 0 <= y < h
        ev = local_dark_evidence(gray, x, y, theta) if inside else 0.0
        strong = ev >= on_thresh
        keep = ev >= keep_thresh
        if strong or (keep and in_run):
            if not in_run:
                in_run = True
                run_start = u
            run_end = u
            gap = 0.0
        elif in_run:
            gap += step
            if gap <= gap_allow:
                run_end = u
            else:
                if run_end is not None and run_end - run_start >= min_run:
                    intervals.append((run_start, run_end - gap + step))
                in_run = False
                run_start = None
                run_end = None
                gap = 0.0
        prev_u = u
        u += step
    if in_run and run_end is not None and run_end - run_start >= min_run:
        intervals.append((run_start, run_end))
    return merge_intervals(intervals, gap=26.0)


def synth_cluster_from_rho(gray, rho, theta):
    intervals = line_scan_intervals(gray, rho, theta)
    total = sum(max(0.0, b - a) for a, b in intervals)
    longest = max([max(0.0, b - a) for a, b in intervals] or [0.0])
    if total < 70.0 or longest < 38.0:
        return None
    h, w = gray.shape[:2]
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    mids = []
    vals = []
    for a, b in intervals:
        for u in np.linspace(a, b, max(3, int((b - a) / 24.0) + 1)):
            x = nx * rho + dx * u
            y = ny * rho + dy * u
            ev = local_dark_evidence(gray, x, y, theta)
            if ev > 0.0 and 0 <= x < w and 0 <= y < h:
                mids.append(y)
                vals.append(ev)
    dark = float(np.mean(vals)) if vals else 0.0
    mid_y = float(np.mean(mids)) if mids else 0.5 * h
    return {
        "rho": float(rho),
        "merged": intervals,
        "total": float(total),
        "longest": float(longest),
        "dark": float(dark),
        "mid_y": float(mid_y),
        "segment_count": 0,
        "score": float(total * (0.25 + min(1.0, dark / 22.0))),
        "synthetic": True,
    }


def densify_primary_run(run, gray, image_shape, theta):
    if len(run) < 2:
        return run
    h, w = image_shape[:2]
    out = list(sorted(run, key=lambda c: c["rho"]))
    # Hardwood plank gaps in this kind of framing tend to sit around ~70-110 px.
    target_spacing = max(58.0, min(108.0, 0.16 * min(h, w)))
    i = 0
    while i < len(out) - 1:
        a = out[i]
        b = out[i + 1]
        gap = b["rho"] - a["rho"]
        if gap > 1.55 * target_spacing:
            inserts = max(1, int(round(gap / target_spacing)) - 1)
            inserted = []
            for k in range(inserts):
                rho = a["rho"] + (k + 1) * gap / (inserts + 1)
                synth = synth_cluster_from_rho(gray, rho, theta)
                if synth is not None:
                    inserted.append(synth)
            if inserted:
                out[i + 1:i + 1] = inserted
                i += len(inserted)
        i += 1
    # Final dedupe / cleanup.
    cleaned = []
    for c in sorted(out, key=lambda c: c["rho"]):
        if cleaned and abs(c["rho"] - cleaned[-1]["rho"]) < 18.0:
            if c["score"] > cleaned[-1]["score"]:
                cleaned[-1] = c
            continue
        cleaned.append(c)
    return cleaned[:10]


def endpoint_supported(gray, x, y, theta):
    vals = []
    dx = math.cos(theta); dy = math.sin(theta)
    for off in (-6.0, -3.0, 0.0, 3.0, 6.0):
        vals.append(local_dark_evidence(gray, x + dx * off, y + dy * off, theta))
    return max(vals) >= 2.2 or (sum(vals) / len(vals)) >= 1.45


def grow_line_by_evidence(gray, line, theta, max_extend=90.0, step=5.0, gap_budget=3):
    x1, y1, x2, y2 = line
    dx = math.cos(theta); dy = math.sin(theta)
    # orient the direction to match the input endpoints, otherwise extension flips.
    if (x2 - x1) * dx + (y2 - y1) * dy < 0:
        dx, dy = -dx, -dy
    h, w = gray.shape[:2]
    def inside(x, y):
        return -8 <= x < w + 8 and -8 <= y < h + 8
    # Extend start backwards.
    gaps = 0
    dist = step
    best_x, best_y = x1, y1
    while dist <= max_extend:
        tx, ty = x1 - dx * dist, y1 - dy * dist
        if not inside(tx, ty): break
        if endpoint_supported(gray, tx, ty, theta):
            best_x, best_y = tx, ty
            gaps = 0
        else:
            gaps += 1
            if gaps > gap_budget: break
        dist += step
    x1, y1 = best_x, best_y
    # Extend end forwards.
    gaps = 0
    dist = step
    best_x, best_y = x2, y2
    while dist <= max_extend:
        tx, ty = x2 + dx * dist, y2 + dy * dist
        if not inside(tx, ty): break
        if endpoint_supported(gray, tx, ty, theta):
            best_x, best_y = tx, ty
            gaps = 0
        else:
            gaps += 1
            if gaps > gap_budget: break
        dist += step
    x2, y2 = best_x, best_y
    return (x1, y1, x2, y2)


def completed_line_from_cluster(c, theta, gray, max_extend=70.0, pad=8.0):
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    intervals = sorted(c["merged"])
    if not intervals:
        return []
    # Use the observed support envelope, but don't bridge arbitrary huge gaps.
    merged = merge_intervals(intervals, gap=42.0)
    out = []
    for a, b in merged[:3]:
        a -= pad; b += pad
        p1 = (nx * c["rho"] + dx * a, ny * c["rho"] + dy * a)
        p2 = (nx * c["rho"] + dx * b, ny * c["rho"] + dy * b)
        out.append(grow_line_by_evidence(gray, (p1[0], p1[1], p2[0], p2[1]), theta, max_extend=max_extend))
    return out

def line_from_cluster(c, theta):
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    # draw the longest merged support interval only plus adjacent intervals if close enough
    intervals = sorted(c["merged"], key=lambda ab: ab[1] - ab[0], reverse=True)
    lines = []
    for a, b in intervals[:2]:
        p1 = (nx * c["rho"] + dx * a, ny * c["rho"] + dy * a)
        p2 = (nx * c["rho"] + dx * b, ny * c["rho"] + dy * b)
        lines.append((p1[0], p1[1], p2[0], p2[1]))
    return lines


def intersect_normal_lines(theta_a, rho_a, theta_b, rho_b):
    a1 = -math.sin(theta_a); b1 = math.cos(theta_a)
    a2 = -math.sin(theta_b); b2 = math.cos(theta_b)
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-5:
        return None
    x = (rho_a * b2 - rho_b * b1) / det
    y = (a1 * rho_b - a2 * rho_a) / det
    return (x, y)


def rho_for_line_mid(line, theta):
    nx = -math.sin(theta); ny = math.cos(theta)
    x1, y1, x2, y2 = line
    return 0.5 * (x1 + x2) * nx + 0.5 * (y1 + y2) * ny


def completed_butt_joint_line(primary, theta, cross_theta, fallback_line, lane, image_shape):
    if lane < 0 or lane + 1 >= len(primary):
        return fallback_line
    rho_cross = rho_for_line_mid(fallback_line, cross_theta)
    p0 = intersect_normal_lines(theta, primary[lane]["rho"], cross_theta, rho_cross)
    p1 = intersect_normal_lines(theta, primary[lane + 1]["rho"], cross_theta, rho_cross)
    if p0 is None or p1 is None:
        return fallback_line
    h, w = image_shape[:2]
    margin = 42.0
    if not (-margin <= p0[0] <= w + margin and -margin <= p0[1] <= h + margin and
            -margin <= p1[0] <= w + margin and -margin <= p1[1] <= h + margin):
        return fallback_line
    return (p0[0], p0[1], p1[0], p1[1])


def line_rect_clip_points(rho, theta, image_shape):
    h, w = image_shape[:2]
    a = -math.sin(theta); b = math.cos(theta)
    pts = []
    for x in (0.0, float(w - 1)):
        if abs(b) > 1e-6:
            y = (rho - a * x) / b
            if 0.0 <= y <= h - 1:
                pts.append((x, y))
    for y in (0.0, float(h - 1)):
        if abs(a) > 1e-6:
            x = (rho - b * y) / a
            if 0.0 <= x <= w - 1:
                pts.append((x, y))
    uniq = []
    for p in pts:
        if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 > 4.0 for q in uniq):
            uniq.append(p)
    if len(uniq) >= 2:
        return uniq[0], uniq[1]
    return None


def seam_profile_candidates(gray, theta):
    """Find full-board seam rhos from a dark-valley profile.

    LSD fragments are good for proving that a seam exists, but they were bad at
    deciding where a complete floorboard edge should be drawn. This 1D profile
    scans across the dominant seam family and finds dark-valley maxima; a later
    DP pass keeps a smooth plank sequence instead of isolated high-score grain.
    """
    h, w = gray.shape[:2]
    diag = math.hypot(w, h)
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    rho_min = -0.48 * diag
    rho_max = 0.44 * diag
    rhos = np.arange(rho_min, rho_max, 1.0)
    means = []
    supports = []
    for rho in rhos:
        vals = []
        support = 0.0
        for u in np.arange(-diag, diag, 4.0):
            x = nx * rho + dx * u
            y = ny * rho + dy * u
            if not (0 <= x < w and 0 <= y < h):
                continue
            c = sample_u8(gray, x, y)
            a = sample_u8(gray, x + nx * 4.5, y + ny * 4.5)
            b = sample_u8(gray, x - nx * 4.5, y - ny * 4.5)
            if c is None or a is None or b is None:
                continue
            ev = max(0.0, 0.5 * (a + b) - c)
            vals.append(ev)
            if ev >= 2.0:
                support += 4.0
        means.append(float(np.mean(vals)) if vals else 0.0)
        supports.append(support)
    means = np.asarray(means, dtype=np.float32)
    supports = np.asarray(supports, dtype=np.float32)
    smooth = np.convolve(means, np.ones(9, dtype=np.float32) / 9.0, mode="same")
    candidates = []
    for i in range(5, len(rhos) - 5):
        if smooth[i] < 2.0 or supports[i] < 40.0:
            continue
        if smooth[i] < np.max(smooth[i - 5:i + 6]):
            continue
        candidates.append({
            "rho": float(rhos[i]),
            "score": float(smooth[i]),
            "support": float(supports[i]),
        })
    return candidates


def choose_smooth_plank_sequence(candidates, preferred_count=9):
    """Choose a coherent row of plank seams from profile peaks.

    This is the key anti-garbage step: several isolated wood-grain peaks can be
    stronger than real seams, so the accepted seams must form a plausible smooth
    plank sequence. Gaps may vary with perspective, but they cannot bounce around
    like random texture.
    """
    candidates = sorted(candidates, key=lambda c: c["rho"])
    if len(candidates) < 4:
        return []
    paths = []

    def visit(path):
        if len(path) >= 6:
            gaps = [path[i + 1]["rho"] - path[i]["rho"] for i in range(len(path) - 1)]
            # Penalize too-dense fake seams and large backward gap changes. A
            # real hardwood run changes spacing smoothly under perspective.
            dense_penalty = sum(max(0.0, 70.0 - g) for g in gaps)
            decrease_penalty = sum(max(0.0, gaps[i] - gaps[i + 1] - 8.0)
                                   for i in range(len(gaps) - 1))
            count_penalty = abs(len(path) - preferred_count)
            score = (
                sum(p["score"] for p in path) +
                10.0 * len(path) -
                6.0 * count_penalty -
                0.12 * dense_penalty -
                0.70 * decrease_penalty
            )
            paths.append((score, list(path)))
        if len(path) >= preferred_count:
            return
        last = path[-1]
        for c in candidates:
            if c["rho"] <= last["rho"]:
                continue
            gap = c["rho"] - last["rho"]
            if gap < 52.0:
                continue
            if gap > 130.0:
                break
            visit(path + [c])

    for c in candidates:
        visit([c])
    if not paths:
        return []
    paths.sort(key=lambda item: item[0], reverse=True)
    return paths[0][1]


def profile_cluster_from_candidate(gray, theta, candidate):
    rho = float(candidate["rho"])
    intervals = line_scan_intervals(gray, rho, theta, step=4.0, on_thresh=1.8, keep_thresh=0.95, min_run=18.0, gap_allow=18.0)
    total = sum(max(0.0, b - a) for a, b in intervals)
    longest = max([max(0.0, b - a) for a, b in intervals] or [0.0])
    h, w = gray.shape[:2]
    nx = -math.sin(theta); ny = math.cos(theta)
    dx = math.cos(theta); dy = math.sin(theta)
    y_samples = []
    for a, b in intervals:
        mid = 0.5 * (a + b)
        y_samples.append(ny * rho + dy * mid)
    return {
        "rho": rho,
        "merged": intervals if intervals else [(-math.hypot(w, h), math.hypot(w, h))],
        "total": max(total, float(candidate.get("support", 0.0))),
        "longest": max(longest, 24.0),
        "dark": float(candidate.get("score", 0.0)),
        "mid_y": float(np.mean(y_samples)) if y_samples else 0.5 * h,
        "segment_count": 1,
        "score": float(candidate.get("score", 0.0)) * max(1.0, float(candidate.get("support", 1.0))),
        "profile_selected": True,
    }


def profile_primary_run(gray, theta):
    sequence = choose_smooth_plank_sequence(seam_profile_candidates(gray, theta), preferred_count=9)
    if len(sequence) < 6:
        return []
    return [profile_cluster_from_candidate(gray, theta, c) for c in sequence]


def snap_primary_rhos_to_real_segments(primary, segment_clusters, max_snap_px=22.0):
    """Snap profile-selected full seams back onto strong real segment clusters.

    The dark-valley profile is good at building a complete plank sequence, but
    it can place a full seam slightly beside an actual high-contrast floor gap.
    That is exactly the failure where the overlay misses an obvious visible line.
    This pass keeps the profile sequence/count, then anchors each seam to the
    nearest real long/dark cluster when the evidence is close enough.
    """
    usable = [c for c in segment_clusters
              if c.get("total", 0.0) >= 120.0 or
                 (c.get("total", 0.0) >= 55.0 and c.get("dark", 0.0) >= 4.2)]
    if not usable:
        return primary
    snapped = []
    used = set()
    for seam in primary:
        best_index = None
        best_cost = float("inf")
        for i, candidate in enumerate(usable):
            if i in used:
                continue
            dist = abs(float(candidate["rho"]) - float(seam["rho"]))
            if dist > max_snap_px:
                continue
            evidence_bonus = min(18.0, 0.018 * float(candidate.get("total", 0.0)) +
                                      1.25 * float(candidate.get("dark", 0.0)))
            cost = dist - evidence_bonus
            if cost < best_cost:
                best_cost = cost
                best_index = i
        if best_index is None:
            snapped.append(seam)
            continue
        real = usable[best_index]
        used.add(best_index)
        anchored = dict(seam)
        anchored["rho"] = float(real["rho"])
        anchored["merged"] = seam.get("merged") or real.get("merged", [])
        anchored["total"] = max(float(seam.get("total", 0.0)), float(real.get("total", 0.0)))
        anchored["longest"] = max(float(seam.get("longest", 0.0)), float(real.get("longest", 0.0)))
        anchored["dark"] = max(float(seam.get("dark", 0.0)), float(real.get("dark", 0.0)))
        anchored["score"] = max(float(seam.get("score", 0.0)), float(real.get("score", 0.0)))
        anchored["snapped_to_segment_cluster"] = True
        snapped.append(anchored)
    snapped.sort(key=lambda c: c["rho"])
    return snapped


def detect_graph(image):
    gray, eq, segs, raw_count = extract_segments(image)
    peaks = orientation_peaks(segs)
    candidates = []
    for theta in peaks:
        refined_theta, clusters, run = refine_orientation(theta, segs, image.shape, gray=gray, want_short=False)
        if len(run) >= 4:
            candidates.append((refined_theta, clusters, run, sum(c["score"] for c in run)))
    if not candidates:
        return {"valid": False, "reason": "no_primary_floorboard_pattern", "raw_segment_count": raw_count, "segment_count": len(segs)}
    # Keep the first viable dominant orientation. The histogram order already says
    # which direction is visually dominant; choosing the highest raw support later
    # can accidentally flip to dense wood grain or short cross-members.
    theta, clusters, primary, _ = candidates[0]
    profile_primary = profile_primary_run(gray, theta)
    if len(profile_primary) >= 6:
        primary = snap_primary_rhos_to_real_segments(profile_primary, clusters)
    # The image-space end-joint family is often not exactly 90 degrees from the
    # primary family after perspective, so choose a real observed secondary peak.
    cross_candidates = []
    for candidate_theta in peaks:
        sep = angle_dist(candidate_theta, theta)
        if sep < math.radians(45.0) or sep > math.radians(135.0):
            continue
        refined_cross, candidate_clusters, short_run = refine_orientation(candidate_theta, segs, image.shape, gray=gray, want_short=True)
        score = sum(c["score"] for c in short_run)
        if score > 0.0:
            cross_candidates.append((score, refined_cross, candidate_clusters))
    if cross_candidates:
        cross_candidates.sort(key=lambda x: x[0], reverse=True)
        cross_theta = cross_candidates[0][1]
        cross_clusters = cross_candidates[0][2]
    else:
        cross_theta = norm_angle(theta + math.pi / 2.0)
        cross_clusters = clusters_for(segs, cross_theta, image.shape)
    # Keep cross clusters that plausibly represent finite butt/end joints: short-to-mid, dark, not giant full-image lines.
    primary_rhos = [c["rho"] for c in primary]
    lanes = []
    for c in cross_clusters:
        lines = line_from_cluster(c, cross_theta)
        if c["total"] < 32.0 or c["total"] > 240.0 or c["dark"] < 3.0:
            continue
        # approximate lane by midpoint projected onto primary normal.
        nxp = -math.sin(theta); nyp = math.cos(theta)
        for line in lines[:1]:
            x1, y1, x2, y2 = line
            mx = 0.5 * (x1 + x2); my = 0.5 * (y1 + y2)
            rho_mid = mx * nxp + my * nyp
            lane = None
            for i in range(len(primary_rhos) - 1):
                lo, hi = sorted((primary_rhos[i], primary_rhos[i + 1]))
                pad = 0.35 * max(1.0, hi - lo)
                if lo - pad <= rho_mid <= hi + pad:
                    lane = i
                    break
            if lane is not None:
                grown = grow_line_by_evidence(gray, line, cross_theta, max_extend=38.0, step=4.0, gap_budget=3)
                snapped = completed_butt_joint_line(primary, theta, cross_theta, grown, lane, image.shape)
                lanes.append((lane, c, snapped))
                break
    # Deduplicate by lane and axial bucket.
    dxt = math.cos(theta); dyt = math.sin(theta)
    spacing_est = float(np.median(np.diff(sorted(primary_rhos)))) if len(primary_rhos) >= 2 else 80.0
    bucket_size = max(30.0, 0.65 * spacing_est)
    kept = []
    seen = set()
    for lane, c, line in sorted(lanes, key=lambda z: z[1]["score"], reverse=True):
        x1, y1, x2, y2 = line
        mx = 0.5 * (x1 + x2); my = 0.5 * (y1 + y2)
        bucket = int(math.floor((mx * dxt + my * dyt) / bucket_size))
        key = (lane, bucket)
        if key in seen:
            continue
        seen.add(key)
        kept.append((lane, bucket, c, line))
        if len(kept) >= 18:
            break
    lane_count = len(set(k[0] for k in kept))
    bucket_count = len(set(k[1] for k in kept))
    valid = len(primary) >= 4 and len(kept) >= 4 and lane_count >= 2 and bucket_count >= 3
    return {
        "valid": valid,
        "reason": "hardwood_planks_pattern_verified_by_profiled_primary_seams_and_cross_board_graph" if valid else "pattern_rejected_sparse_primary_or_cross_board_graph",
        "raw_segment_count": raw_count,
        "segment_count": len(segs),
        "pattern_type": "hardwood_planks" if valid else "unknown_floor_texture",
        "orientation_deg": round(math.degrees(theta), 2),
        "cross_orientation_deg": round(math.degrees(cross_theta), 2),
        "major_seam_count": len(primary),
        "secondary_seam_count": len(kept),
        "butt_joint_lane_count": lane_count,
        "butt_joint_axial_bucket_count": bucket_count,
        "spacing_px": spacing_est,
        "confidence": clamp01(0.28 + 0.08 * len(primary) + 0.04 * len(kept) + 0.08 * lane_count + 0.04 * bucket_count),
        "primary_rhos": [round(float(c["rho"]), 2) for c in primary],
        "snapped_primary_count": sum(1 for c in primary if c.get("snapped_to_segment_cluster")),
        "major_clusters": primary,
        "butt_joints": kept,
    }


def draw_overlay(image, result, out_path):
    """Render the accepted board layout.

    This is intentionally clean now: cyan lines on the original image. The old
    debug renderer used a green mask, labels, and separate magenta butt-joints,
    which made a good layout look worse and hid whether the detected seams
    actually matched the floorboards.
    """
    overlay = image.copy()
    h, w = image.shape[:2]

    def clip_point(x, y):
        return (int(round(max(0.0, min(float(w - 1), x)))),
                int(round(max(0.0, min(float(h - 1), y)))))

    if result.get("valid"):
        theta = math.radians(result["orientation_deg"])
        for c in result["major_clusters"]:
            clipped = line_rect_clip_points(c["rho"], theta, image.shape)
            if clipped is None:
                continue
            p0, p1 = clipped
            cv2.line(overlay, clip_point(*p0), clip_point(*p1), (255, 255, 0), 3, cv2.LINE_AA)
        for lane, bucket, c, line in result["butt_joints"]:
            cv2.line(overlay,
                     clip_point(line[0], line[1]),
                     clip_point(line[2], line[3]),
                     (255, 255, 0), 3, cv2.LINE_AA)
    else:
        status = "rejected"
        cv2.putText(overlay, f"{status} pattern={result.get('pattern_type')} conf={result.get('confidence',0):.2f}",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (60, 60, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, result.get("reason", "")[:90],
                    (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(out_path), overlay)


def draw_debug_overlay(image, result, out_path):
    overlay = image.copy()
    h, w = image.shape[:2]

    def clip_point(x, y):
        return [int(round(max(0.0, min(float(w - 1), x)))),
                int(round(max(0.0, min(float(h - 1), y))))]

    if result.get("valid"):
        theta = math.radians(result["orientation_deg"])
        pts = []
        for c in result["major_clusters"]:
            clipped = line_rect_clip_points(c["rho"], theta, image.shape)
            if clipped is None:
                continue
            p0, p1 = clipped
            pts.append(clip_point(*p0))
            pts.append(clip_point(*p1))
        if len(pts) >= 3:
            hull = cv2.convexHull(np.array(pts, dtype=np.float32)).astype(np.int32)
            mask = overlay.copy()
            cv2.fillConvexPoly(mask, hull, (60, 210, 80))
            overlay = cv2.addWeighted(mask, 0.18, overlay, 0.82, 0)
            cv2.polylines(overlay, [hull], True, (60, 235, 80), 2, cv2.LINE_AA)
        for c in result["major_clusters"]:
            clipped = line_rect_clip_points(c["rho"], theta, image.shape)
            if clipped is None:
                continue
            p0, p1 = clipped
            cv2.line(overlay, tuple(clip_point(*p0)), tuple(clip_point(*p1)), (255, 255, 0), 3, cv2.LINE_AA)
        for lane, bucket, c, line in result["butt_joints"]:
            cv2.line(overlay,
                     tuple(clip_point(line[0], line[1])),
                     tuple(clip_point(line[2], line[3])),
                     (255, 80, 255), 3, cv2.LINE_AA)
    status = "accepted" if result.get("valid") else "rejected"
    cv2.putText(overlay, f"{status} pattern={result.get('pattern_type')} conf={result.get('confidence',0):.2f}",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (30, 255, 80) if result.get("valid") else (60, 60, 255), 2, cv2.LINE_AA)
    cv2.putText(overlay, f"major={result.get('major_seam_count',0)} butt={result.get('secondary_seam_count',0)} "
                         f"lanes={result.get('butt_joint_lane_count',0)} buckets={result.get('butt_joint_axial_bucket_count',0)}",
                (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(out_path), overlay)


def json_safe(result):
    out = dict(result)
    out.pop("major_clusters", None)
    out.pop("butt_joints", None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--out-dir", default="floor_pattern_probe_out")
    args = ap.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"failed to read image: {args.image}")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    result = detect_graph(image)
    stem = Path(args.image).stem
    overlay_path = out_dir / f"{stem}_graph_overlay.png"
    debug_overlay_path = out_dir / f"{stem}_graph_debug_overlay.png"
    json_path = out_dir / f"{stem}_graph.json"
    draw_overlay(image, result, overlay_path)
    draw_debug_overlay(image, result, debug_overlay_path)
    safe = json_safe(result)
    safe["overlay"] = str(overlay_path)
    safe["debug_overlay"] = str(debug_overlay_path)
    safe["report"] = str(json_path)
    json_path.write_text(json.dumps(safe, indent=2), encoding="utf-8")
    print(json.dumps(safe, indent=2))
    return 0 if result.get("valid") else 2


if __name__ == "__main__":
    raise SystemExit(main())
