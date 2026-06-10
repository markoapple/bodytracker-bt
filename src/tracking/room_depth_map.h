#pragma once

#include "tracking/anchor_space_mapper.h"

#include <vector>

namespace bt {

class RoomDepthMap {
public:
    void Configure(RoomDepthMapConfig config) {
        config_ = config;
        telemetry_ = RoomDepthMapTelemetry{};
        telemetry_.state = config_.enabled ? "warming_up" : "disabled";
        telemetry_.last_rejection_reason = config_.enabled ? "not_enough_samples" : "disabled";
        cells_.assign(static_cast<std::size_t>(std::max(0, config_.resolution_width * config_.resolution_height)), RoomDepthCell{});
    }

    [[nodiscard]] const RoomDepthMapConfig& Config() const noexcept { return config_; }
    [[nodiscard]] const RoomDepthMapTelemetry& Telemetry() const noexcept { return telemetry_; }

    void ObserveAnchorCorrection(const ProjectionCorrection& correction, double now_seconds) {
        telemetry_ = UpdateRoomDepthMapTelemetry(telemetry_, config_, correction, now_seconds);
    }

    [[nodiscard]] RoomDepthSample Sample(const Vec2f& pixel, int image_width, int image_height) const {
        RoomDepthSample out;
        if (!config_.enabled || config_.collect_only || telemetry_.state != "active" ||
            image_width <= 0 || image_height <= 0 || cells_.empty() ||
            !PixelFiniteInside(pixel, image_width, image_height)) {
            return out;
        }
        const int x = std::clamp(static_cast<int>(pixel.x * static_cast<float>(config_.resolution_width) / static_cast<float>(image_width)), 0, std::max(0, config_.resolution_width - 1));
        const int y = std::clamp(static_cast<int>(pixel.y * static_cast<float>(config_.resolution_height) / static_cast<float>(image_height)), 0, std::max(0, config_.resolution_height - 1));
        const auto& cell = cells_[static_cast<std::size_t>(y * config_.resolution_width + x)];
        if (cell.valid && cell.sample_count >= static_cast<std::uint32_t>(config_.min_samples_per_cell) &&
            cell.variance_m2 <= config_.max_cell_variance_m2) {
            out.valid = true;
            out.depth_m = cell.depth_m;
            out.variance_m2 = cell.variance_m2;
            out.sample_count = cell.sample_count;
        }
        return out;
    }

private:
    RoomDepthMapConfig config_{};
    RoomDepthMapTelemetry telemetry_{};
    std::vector<RoomDepthCell> cells_{};
};

} // namespace bt
