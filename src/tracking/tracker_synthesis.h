#pragma once

#include "calibration/calibration_types.h"
#include "core/types.h"

#include <array>
#include <cstddef>

namespace bt {

struct LowerBodyModel;
struct UnifiedBodyState;

enum class TrackerRole {
    Pelvis = 0,
    LeftFoot,
    RightFoot,
    Chest,
    LeftElbow,
    RightElbow,
    LeftKnee,
    RightKnee
};

inline constexpr std::size_t kTrackerPoseCount = 8;
inline constexpr std::array<TrackerRole, kTrackerPoseCount> kTrackerRoles{
    TrackerRole::Pelvis,
    TrackerRole::LeftFoot,
    TrackerRole::RightFoot,
    TrackerRole::Chest,
    TrackerRole::LeftElbow,
    TrackerRole::RightElbow,
    TrackerRole::LeftKnee,
    TrackerRole::RightKnee
};

inline std::size_t TrackerRoleIndex(TrackerRole role) {
    switch (role) {
    case TrackerRole::Pelvis: return 0;
    case TrackerRole::LeftFoot: return 1;
    case TrackerRole::RightFoot: return 2;
    case TrackerRole::Chest: return 3;
    case TrackerRole::LeftElbow: return 4;
    case TrackerRole::RightElbow: return 5;
    case TrackerRole::LeftKnee: return 6;
    case TrackerRole::RightKnee: return 7;
    default: return kTrackerPoseCount;
    }
}

inline const char* ToString(TrackerRole role) {
    switch (role) {
    case TrackerRole::Pelvis: return "pelvis";
    case TrackerRole::LeftFoot: return "left_foot";
    case TrackerRole::RightFoot: return "right_foot";
    case TrackerRole::Chest: return "chest";
    case TrackerRole::LeftElbow: return "left_elbow";
    case TrackerRole::RightElbow: return "right_elbow";
    case TrackerRole::LeftKnee: return "left_knee";
    case TrackerRole::RightKnee: return "right_knee";
    default: return "unknown";
    }
}

struct TrackerPose {
    TrackerRole role = TrackerRole::Pelvis;
    Pose3f pose{};
    float confidence = 0.0f;
    bool valid = false;
    TrackerEvidence evidence{};
};

using TrackerPoseArray = std::array<TrackerPose, kTrackerPoseCount>;

TrackerPoseArray SynthesizeTrackerPoses(const LowerBodyState& state);
TrackerPoseArray SynthesizeTrackerPoses(const LowerBodyState& state, const LowerBodyModel& model);
TrackerPoseArray SynthesizeTrackerPoses(const UnifiedBodyState& body_state, const LowerBodyModel& model);
void MarkTrackersStereoFallback(TrackerPoseArray& trackers);

} // namespace bt
