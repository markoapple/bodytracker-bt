\
#!/usr/bin/env python3
"""
Conservative smoke visualizer for the v24 hardwood failure case.

Acceptance target:
finite visible seam segments only, no global model redraw.
"""
from __future__ import annotations

import math
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "generated" / "v24_hardwood_visible_segments" / "hardwood_source.png"
OUT = ROOT / "generated" / "v24_hardwood_visible_segments" / "hardwood_v24_visible_segments_overlay_regenerated.png"

def sample_bilinear(im: np.ndarray, x: float, y: float):
    h, w = im.shape[:2]
    if x < 0 or y < 0 or x + 1 >= w or y + 1 >= h:
        return None
    x0 = int(math.floor(x))
    y0 = int(math.floor(y))
    tx = x - x0
    ty = y - y0
    v00 = float(im[y0, x0])
    v10 = float(im[y0, x0 + 1])
    v01 = float(im[y0 + 1, x0])
    v11 = float(im[y0 + 1, x0 + 1])
    return (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty

def dark_valley_stats(eq: np.ndarray, x1: float, y1: float, x2: float, y2: float):
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length < 1:
        return None
    nx = -dy / length
    ny = dx / length
    n = max(8, min(64, int(round(length / 8))))
    good = 0
    vals = []
    oks = []
    for i in range(n):
        t = (i + 0.5) / n
        bx = x1 + t * dx
        by = y1 + t * dy
        best = None
        for off in np.linspace(-4, 4, 9):
            x = bx + nx * off
            y = by + ny * off
            c = sample_bilinear(eq, x, y)
            if c is None:
                continue
            samples = [
                sample_bilinear(eq, x + nx * 3, y + ny * 3),
                sample_bilinear(eq, x - nx * 3, y - ny * 3),
                sample_bilinear(eq, x + nx * 7, y + ny * 7),
                sample_bilinear(eq, x - nx * 7, y - ny * 7),
            ]
            if any(v is None for v in samples):
                continue
            a1, b1, a2, b2 = samples
            side = (a1 + b1 + a2 + b2) / 4
            valley = side - c
            balance = abs((a1 + a2) / 2 - (b1 + b2) / 2)
            score = valley - 0.08 * balance
            if best is None or score > best[0]:
                best = (score, valley, balance)
        if best is None:
            oks.append(False)
            continue
        _, valley, balance = best
        vals.append(valley)
        ok = valley >= 5.5 and balance <= 45
        oks.append(ok)
        if ok:
            good += 1
    if not vals:
        return None
    best_run = cur = 0
    for ok in oks:
        if ok:
            cur += 1
            best_run = max(best_run, cur)
        else:
            cur = 0
    return good / n, best_run / n, float(np.mean(vals))

def main() -> int:
    img = cv2.imread(str(SRC))
    if img is None:
        raise SystemExit(f"missing {SRC}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    eq = cv2.createCLAHE(clipLimit=1.8, tileGridSize=(8, 8)).apply(gray)
    eq = cv2.GaussianBlur(eq, (3, 3), 0.5)

    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_ADV)
    raw = lsd.detect(eq)[0]
    segs = []

    if raw is not None:
        for line in raw[:, 0, :]:
            x1, y1, x2, y2 = map(float, line)
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 42:
                continue
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180
            if 12 <= angle <= 45:
                family = "long"
            elif 100 <= angle <= 150:
                family = "joint"
            else:
                continue
            stats = dark_valley_stats(eq, x1, y1, x2, y2)
            if stats is None:
                continue
            good_ratio, run_ratio, mean_valley = stats
            if family == "long":
                if length < 70 or good_ratio < 0.50 or run_ratio < 0.34:
                    continue
            else:
                if length < 45 or good_ratio < 0.45 or run_ratio < 0.28:
                    continue
            if mean_valley < 7:
                continue
            score = length * (0.6 * good_ratio + 0.4 * run_ratio) * min(2.0, mean_valley / 10)
            segs.append((score, family, x1, y1, x2, y2, length, angle))

    segs.sort(reverse=True, key=lambda row: row[0])
    kept = []

    def rho_u(seg):
        _, _, x1, y1, x2, y2, _, angle = seg
        theta = math.radians(angle)
        nx = -math.sin(theta)
        ny = math.cos(theta)
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2
        rho = mx * nx + my * ny
        u0 = x1 * math.cos(theta) + y1 * math.sin(theta)
        u1 = x2 * math.cos(theta) + y2 * math.sin(theta)
        return rho, min(u0, u1), max(u0, u1), angle

    for seg in segs:
        rho, u0, u1, angle = rho_u(seg)
        duplicate = False
        for old in kept:
            old_rho, old_u0, old_u1, old_angle = rho_u(old)
            if seg[1] == old[1] and abs(rho - old_rho) < 7 and abs(angle - old_angle) < 4:
                overlap = max(0.0, min(u1, old_u1) - max(u0, old_u0))
                if overlap > 0.35 * min(u1 - u0, old_u1 - old_u0):
                    duplicate = True
                    break
        if not duplicate:
            kept.append(seg)

    overlay = img.copy()
    for _, family, x1, y1, x2, y2, _, _ in kept:
        color = (0, 255, 0) if family == "long" else (0, 215, 255)
        cv2.line(overlay, (round(x1), round(y1)), (round(x2), round(y2)), color, 2, cv2.LINE_AA)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT), overlay)
    long_count = sum(1 for row in kept if row[1] == "long")
    joint_count = sum(1 for row in kept if row[1] == "joint")
    print(f"kept_visible_seams={len(kept)} long={long_count} joints={joint_count}")
    print("Graph sanity from this overlay: finite segments are grouped after detection; no full-line redraw is performed.")
    print(f"wrote={OUT}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
