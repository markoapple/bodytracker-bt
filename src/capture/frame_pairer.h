#pragma once

#include "capture/frame.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bt {

struct FramePairerConfig {
    double max_skew_ms = 18.0;
};

struct PairedFrames {
    std::shared_ptr<const FramePacket> camera_a;
    std::shared_ptr<const FramePacket> camera_b;
    double skew_ms = 0.0;
    bool valid = false;
    bool degraded = false;
    bool reused_a = false;
    bool reused_b = false;
    bool duplicate = false;
    bool skewed = false;
    std::string reason;
};

struct FramePairerTelemetry {
    std::uint64_t accepted_pairs = 0;
    std::uint64_t missing_a = 0;
    std::uint64_t missing_b = 0;
    std::uint64_t rejected_skew = 0;
    std::uint64_t rejected_duplicate = 0;
    std::uint64_t rejected_reused_a = 0;
    std::uint64_t rejected_reused_b = 0;
    std::uint64_t degraded_skew = 0;
    std::uint64_t degraded_duplicate = 0;
    std::uint64_t degraded_reused_a = 0;
    std::uint64_t degraded_reused_b = 0;
    std::uint64_t last_accepted_sequence_a = 0;
    std::uint64_t last_accepted_sequence_b = 0;
    double last_skew_ms = 0.0;
};

class FramePairer {
public:
    explicit FramePairer(FramePairerConfig config = {});

    PairedFrames Pair(
        std::shared_ptr<const FramePacket> camera_a,
        std::shared_ptr<const FramePacket> camera_b);

    PairedFrames PairRecent(
        const std::vector<std::shared_ptr<const FramePacket>>& camera_a,
        const std::vector<std::shared_ptr<const FramePacket>>& camera_b);

    [[nodiscard]] const FramePairerTelemetry& Telemetry() const noexcept;

private:
    FramePairerConfig config_;
    FramePairerTelemetry telemetry_{};
    std::uint64_t last_accepted_sequence_a_ = 0;
    std::uint64_t last_accepted_sequence_b_ = 0;
    bool has_last_accepted_pair_ = false;
};

} // namespace bt
