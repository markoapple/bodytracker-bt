#pragma once

#include "core/timing.h"

#include <cstdint>
#include <string>

namespace bt {

struct CaptureHealthSnapshot {
    bool opened = false;
    bool running = false;
    int actual_width = 0;
    int actual_height = 0;
    double actual_fps = 0.0;
    double last_read_ms = 0.0;
    int backend_api = 0;
    std::string backend_name;
    std::uint64_t open_failures = 0;
    std::uint64_t delivered_frames = 0;
    std::uint64_t read_failures = 0;
    std::uint64_t consecutive_read_failures = 0;
    std::uint64_t slot_replacements = 0;
    QpcTimestamp last_frame_timestamp{};
    double last_frame_age_ms = 0.0;
    double network_jitter_ms = 0.0;
    std::uint64_t network_reconnect_count = 0;
    std::uint64_t network_accept_count = 0;
    std::uint64_t network_stream_header_count = 0;
    std::uint64_t network_stream_header_failures = 0;
    std::uint64_t network_frame_header_count = 0;
    std::uint64_t network_frame_header_failures = 0;
    std::uint64_t network_frame_payload_count = 0;
    std::uint64_t network_frame_payload_failures = 0;
    std::uint64_t network_bad_frame_size_count = 0;
    std::uint64_t network_decode_successes = 0;
    std::uint64_t network_decode_failures = 0;
    std::string source_state = "stopped";
    std::string last_frame_status = "no_frame";
    std::string last_health_message;
    std::string last_degraded_reason;
    std::string last_decode_error;
    std::string last_error_message;
};

inline std::string NetworkCaptureCountsSummary(const CaptureHealthSnapshot& h) {
    return "network counts accept=" + std::to_string(h.network_accept_count) +
        " stream_header=" + std::to_string(h.network_stream_header_count) +
        " stream_header_fail=" + std::to_string(h.network_stream_header_failures) +
        " frame_header=" + std::to_string(h.network_frame_header_count) +
        " frame_header_fail=" + std::to_string(h.network_frame_header_failures) +
        " payload=" + std::to_string(h.network_frame_payload_count) +
        " payload_fail=" + std::to_string(h.network_frame_payload_failures) +
        " bad_size=" + std::to_string(h.network_bad_frame_size_count) +
        " decoded=" + std::to_string(h.network_decode_successes) +
        " decode_fail=" + std::to_string(h.network_decode_failures) +
        " delivered=" + std::to_string(h.delivered_frames) +
        " read_fail=" + std::to_string(h.read_failures) +
        " reconnect=" + std::to_string(h.network_reconnect_count);
}

inline void RefreshNetworkCaptureHealthMessage(CaptureHealthSnapshot& h) {
    h.last_health_message = NetworkCaptureCountsSummary(h);
}

} // namespace bt
