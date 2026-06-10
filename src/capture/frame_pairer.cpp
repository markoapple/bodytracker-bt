#include "capture/frame_pairer.h"

#include <cmath>
#include <limits>
#include <utility>

namespace bt {
namespace {

enum class PairQuality : int {
    Fresh = 0,
    ReusedOneCamera = 1,
    Duplicate = 2,
    Skewed = 3,
};

const char* PairReason(PairQuality quality, bool reused_a, bool reused_b, bool skewed) {
    if (skewed) {
        return "timestamp_skew_degraded";
    }
    if (quality == PairQuality::Duplicate) {
        return "duplicate_pair_degraded";
    }
    if (reused_a && reused_b) {
        return "duplicate_pair_degraded";
    }
    if (reused_a) {
        return "reused_camera_a_frame_degraded";
    }
    if (reused_b) {
        return "reused_camera_b_frame_degraded";
    }
    return "paired";
}

} // namespace

FramePairer::FramePairer(FramePairerConfig config)
    : config_(config) {
}

PairedFrames FramePairer::Pair(
    std::shared_ptr<const FramePacket> camera_a,
    std::shared_ptr<const FramePacket> camera_b) {

    std::vector<std::shared_ptr<const FramePacket>> a;
    std::vector<std::shared_ptr<const FramePacket>> b;
    if (camera_a) {
        a.push_back(std::move(camera_a));
    }
    if (camera_b) {
        b.push_back(std::move(camera_b));
    }
    return PairRecent(a, b);
}

PairedFrames FramePairer::PairRecent(
    const std::vector<std::shared_ptr<const FramePacket>>& camera_a,
    const std::vector<std::shared_ptr<const FramePacket>>& camera_b) {

    PairedFrames best;
    if (camera_a.empty()) {
        telemetry_.missing_a += 1;
        best.reason = "missing_camera_a";
        return best;
    }
    if (camera_b.empty()) {
        telemetry_.missing_b += 1;
        best.reason = "missing_camera_b";
        return best;
    }

    double best_score = std::numeric_limits<double>::infinity();
    double best_skew = std::numeric_limits<double>::infinity();
    bool best_reused_a = false;
    bool best_reused_b = false;
    bool best_skewed = false;
    PairQuality best_quality = PairQuality::Skewed;

    for (const auto& a : camera_a) {
        if (!a) {
            continue;
        }
        for (const auto& b : camera_b) {
            if (!b) {
                continue;
            }
            const double skew = 1000.0 * std::abs(QpcDeltaSeconds(a->timestamp, b->timestamp));
            if (skew < best_skew) {
                best_skew = skew;
                telemetry_.last_skew_ms = skew;
            }

            const bool reused_a = has_last_accepted_pair_ && a->sequence == last_accepted_sequence_a_;
            const bool reused_b = has_last_accepted_pair_ && b->sequence == last_accepted_sequence_b_;
            const bool duplicate = reused_a && reused_b;
            const bool skewed = skew > config_.max_skew_ms;
            PairQuality quality = PairQuality::Fresh;
            if (skewed) {
                quality = PairQuality::Skewed;
            } else if (duplicate) {
                quality = PairQuality::Duplicate;
            } else if (reused_a || reused_b) {
                quality = PairQuality::ReusedOneCamera;
            }

            // Lower score wins: prefer truly fresh synchronized pairs, but do not
            // throw away finite older pixels merely because one side is reused or
            // the clocks are imperfect. The quality label becomes telemetry and
            // confidence shaping downstream, not a veto.
            const double score = static_cast<int>(quality) * 100000.0 + skew;
            if (score < best_score) {
                best_score = score;
                best.camera_a = a;
                best.camera_b = b;
                best.skew_ms = skew;
                best.valid = true;
                best.degraded = quality != PairQuality::Fresh;
                best.reused_a = reused_a;
                best.reused_b = reused_b;
                best.duplicate = duplicate;
                best.skewed = skewed;
                best.reason = PairReason(quality, reused_a, reused_b, skewed);
                best_quality = quality;
                best_reused_a = reused_a;
                best_reused_b = reused_b;
                best_skewed = skewed;
            }
        }
    }

    if (!best.valid) {
        best.reason = "no_pair_candidate";
        best.skew_ms = std::isfinite(best_skew) ? best_skew : 0.0;
        return best;
    }

    last_accepted_sequence_a_ = best.camera_a->sequence;
    last_accepted_sequence_b_ = best.camera_b->sequence;
    has_last_accepted_pair_ = true;
    telemetry_.last_accepted_sequence_a = last_accepted_sequence_a_;
    telemetry_.last_accepted_sequence_b = last_accepted_sequence_b_;
    telemetry_.last_skew_ms = best.skew_ms;
    telemetry_.accepted_pairs += 1;
    if (best_skewed) {
        telemetry_.degraded_skew += 1;
    } else if (best_quality == PairQuality::Duplicate) {
        telemetry_.degraded_duplicate += 1;
    } else if (best_reused_a) {
        telemetry_.degraded_reused_a += 1;
    } else if (best_reused_b) {
        telemetry_.degraded_reused_b += 1;
    }
    if (best.reason.empty()) {
        best.reason = "paired";
    }
    return best;
}

const FramePairerTelemetry& FramePairer::Telemetry() const noexcept {
    return telemetry_;
}

} // namespace bt
