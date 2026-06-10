#include "tracking/reliability.h"
#include "test_check.h"

#include <cmath>
#include <cstdint>

namespace {

struct Lcg {
    std::uint32_t state = 0x9e3779b9u;
    std::uint32_t NextU32() {
        state = state * 1664525u + 1013904223u;
        return state;
    }
    float Unit() {
        return static_cast<float>(NextU32() & 0x00ffffffu) / static_cast<float>(0x00ffffffu);
    }
    float Range(float lo, float hi) {
        return lo + (hi - lo) * Unit();
    }
};

bt::DecodedPose2D RandomPose(Lcg& rng, int width, int height) {
    bt::DecodedPose2D pose;
    pose.valid = true;
    pose.aggregate_confidence = rng.Unit();
    for (auto& kp : pose.keypoints) {
        kp.present = (rng.NextU32() % 5u) != 0u;
        kp.confidence = rng.Range(-0.1f, 1.1f);
        kp.pixel = bt::Vec2f{rng.Range(-40.0f, static_cast<float>(width) + 40.0f), rng.Range(-40.0f, static_cast<float>(height) + 40.0f)};
    }
    return pose;
}

bt::RoiState RandomRoi(Lcg& rng, int width, int height) {
    bt::RoiState roi;
    // This fuzz test checks successful reliability output invariants. Invalid ROI
    // rejection is covered by direct validation tests; mixing it here made the
    // test fail for the wrong reason.
    roi.initialized = true;
    roi.rect = bt::Rect2f{
        rng.Range(-20.0f, static_cast<float>(width) * 0.4f),
        rng.Range(-20.0f, static_cast<float>(height) * 0.4f),
        rng.Range(1.0f, static_cast<float>(width)),
        rng.Range(1.0f, static_cast<float>(height))};
    return roi;
}

} // namespace

int main() {
    Lcg rng;
    for (int i = 0; i < 4096; ++i) {
        const int width = 160 + static_cast<int>(rng.NextU32() % 1760u);
        const int height = 120 + static_cast<int>(rng.NextU32() % 960u);
        const auto current = RandomPose(rng, width, height);
        const auto previous = RandomPose(rng, width, height);
        const auto roi = RandomRoi(rng, width, height);
        const auto result = bt::ComputeViewReliability(
            current,
            roi,
            width,
            height,
            bt::PostureMode::UprightStanding,
            (rng.NextU32() % 2u) ? &previous : nullptr);
        BT_CHECK(result.ok());
        const auto& summary = result.value();
        BT_CHECK(std::isfinite(summary.mean_weight));
        BT_CHECK(std::isfinite(summary.lower_body_mean));
        BT_CHECK(std::isfinite(summary.foot_mean));
        BT_CHECK(summary.mean_weight >= 0.0f && summary.mean_weight <= 1.0f);
        BT_CHECK(summary.lower_body_mean >= 0.0f && summary.lower_body_mean <= 1.0f);
        BT_CHECK(summary.foot_mean >= 0.0f && summary.foot_mean <= 1.0f);
        for (const auto& joint : summary.joints) {
            BT_CHECK(std::isfinite(joint.final_weight));
            BT_CHECK(joint.final_weight >= 0.0f && joint.final_weight <= 1.0f);
            BT_CHECK(std::isfinite(joint.temporal_term));
            BT_CHECK(joint.temporal_term >= 0.0f && joint.temporal_term <= 1.0f);
        }
    }
    return 0;
}
