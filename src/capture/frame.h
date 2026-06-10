#pragma once

#include "core/timing.h"

#include <cstdint>
#include <opencv2/core.hpp>

namespace bt {

struct FramePacket {
    cv::Mat bgr;
    int width = 0;
    int height = 0;
    std::uint64_t sequence = 0;
    QpcTimestamp timestamp{};
};

} // namespace bt
