#include "tracking/reliability.h"

#include "inference/keypoint_contract.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace bt {
namespace {

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

float SmoothStep01(float x) {
    x = Clamp01(x);
    return x * x * (3.0f - 2.0f * x);
}

float EdgeDistance(const Rect2f& r, const Vec2f& p) {
    return std::min(std::min(p.x - r.x, r.x + r.width - p.x), std::min(p.y - r.y, r.y + r.height - p.y));
}

float CropStability(const RoiState& roi) {
    const float base = std::max(1.0f, std::min(roi.rect.width, roi.rect.height));
    const float shift = roi.diagnostics.center_shift_px / base;
    const float sx = std::max(1e-3f, roi.diagnostics.scale_x_ratio);
    const float sy = std::max(1e-3f, roi.diagnostics.scale_y_ratio);
    float term = 1.0f / (1.0f + 4.0f * shift + 6.0f * std::max(std::abs(std::log(sx)), std::abs(std::log(sy))));
    if (roi.in_reacquire) {
        term *= 0.85f;
    }
    return Clamp01(std::max(0.30f, term));
}

float PostureModeTerm(PostureMode mode, KeypointId id) {
    if ((mode == PostureMode::SeatedSupported || mode == PostureMode::ReclinedSupported) && IsFootKeypoint(id)) {
        return 0.85f;
    }
    return 1.0f;
}

} // namespace

Result<ReliabilitySummary> ComputeViewReliability(
    const DecodedPose2D& current_pose,
    const RoiState& roi,
    int frame_width,
    int frame_height,
    PostureMode posture_mode,
    const DecodedPose2D* previous_pose,
    const ReliabilityConfig& config) {

    if (!current_pose.valid) {
        return Status::Error(StatusCode::InvalidArgument, "ComputeViewReliability requires a valid pose");
    }
    if (!roi.initialized || roi.rect.width <= 0.0f || roi.rect.height <= 0.0f) {
        return Status::Error(StatusCode::InvalidArgument, "ComputeViewReliability requires a valid ROI");
    }

    ReliabilitySummary summary;
    const float crop_stability = CropStability(roi);
    const Rect2f image{0.0f, 0.0f, static_cast<float>(frame_width), static_cast<float>(frame_height)};

    float sum = 0.0f;
    int count = 0;
    float lower_sum = 0.0f;
    int lower_count = 0;
    float foot_sum = 0.0f;
    int foot_count = 0;

    for (std::size_t i = 0; i < kHalpe26Count; ++i) {
        const auto id = static_cast<KeypointId>(i);
        const auto& kp = current_pose.keypoints[i];
        auto& jr = summary.joints[i];
        if (!kp.present || kp.confidence < config.min_present_confidence) {
            continue;
        }

        jr.model_term = std::sqrt(Clamp01(kp.confidence));
        jr.crop_edge_term = SmoothStep01(EdgeDistance(roi.rect, kp.pixel) / std::max(8.0f, config.crop_edge_soft_fraction * std::min(roi.rect.width, roi.rect.height)));
        jr.image_edge_term = SmoothStep01(EdgeDistance(image, kp.pixel) / std::max(1.0f, config.image_edge_soft_px));
        jr.crop_stability_term = crop_stability;
        jr.posture_mode_term = PostureModeTerm(posture_mode, id);

        if (previous_pose && previous_pose->valid && previous_pose->keypoints[i].present) {
            const float sigma = std::max(6.0f, config.temporal_sigma_fraction * std::max(roi.rect.width, roi.rect.height));
            const float d = Distance(kp.pixel, previous_pose->keypoints[i].pixel);
            jr.temporal_term = std::max(config.temporal_floor, std::exp(-0.5f * (d * d) / (sigma * sigma)));
            jr.temporal_computed = true;
        }

        float weight =
            jr.model_term *
            jr.crop_edge_term *
            jr.image_edge_term *
            jr.crop_stability_term *
            jr.posture_mode_term;
        if (jr.temporal_computed) {
            weight *= jr.temporal_term;
        }
        jr.final_weight = Clamp01(weight);
        jr.usable = jr.final_weight >= 0.15f;

        sum += jr.final_weight;
        ++count;
        if (IsLowerBodyKeypoint(id)) {
            lower_sum += jr.final_weight;
            ++lower_count;
        }
        if (IsFootKeypoint(id)) {
            foot_sum += jr.final_weight;
            ++foot_count;
        }
    }

    summary.mean_weight = count > 0 ? sum / static_cast<float>(count) : 0.0f;
    summary.lower_body_mean = lower_count > 0 ? lower_sum / static_cast<float>(lower_count) : 0.0f;
    summary.foot_mean = foot_count > 0 ? foot_sum / static_cast<float>(foot_count) : 0.0f;
    return summary;
}

std::string BuildReliabilitySummary(const ReliabilitySummary& summary) {
    std::ostringstream oss;
    oss << "reliability mean=" << summary.mean_weight
        << " lower=" << summary.lower_body_mean
        << " foot=" << summary.foot_mean;
    return oss.str();
}

} // namespace bt
