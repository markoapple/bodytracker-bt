#pragma once

// Internal 26-keypoint contract. This is the solver's canonical joint topology,
// NOT a model-specific format. The Cocktail14 133-keypoint whole-body model
// maps to this 26-keypoint internal representation via MapWholeBody133ToInternal26().
// All solver, support, and tracking code uses these KeypointId indices.
// See also: rtmpose_decode.cpp for the 133->26 mapping.

#include "core/types.h"

#include <array>
#include <cstddef>

namespace bt {

inline constexpr std::array<KeypointId, kInternalKeypointCount> kInternalKeypointOrder{
    KeypointId::Nose,
    KeypointId::LeftEye,
    KeypointId::RightEye,
    KeypointId::LeftEar,
    KeypointId::RightEar,
    KeypointId::LeftShoulder,
    KeypointId::RightShoulder,
    KeypointId::LeftElbow,
    KeypointId::RightElbow,
    KeypointId::LeftWrist,
    KeypointId::RightWrist,
    KeypointId::LeftHip,
    KeypointId::RightHip,
    KeypointId::LeftKnee,
    KeypointId::RightKnee,
    KeypointId::LeftAnkle,
    KeypointId::RightAnkle,
    KeypointId::HeadTop,
    KeypointId::Neck,
    KeypointId::Pelvis,
    KeypointId::LeftBigToe,
    KeypointId::RightBigToe,
    KeypointId::LeftSmallToe,
    KeypointId::RightSmallToe,
    KeypointId::LeftHeel,
    KeypointId::RightHeel,
};

inline constexpr std::array<const char*, kInternalKeypointCount> kInternalKeypointNames{
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "head",
    "neck",
    "hip",
    "left_big_toe",
    "right_big_toe",
    "left_small_toe",
    "right_small_toe",
    "left_heel",
    "right_heel",
};

inline constexpr std::array<KeypointId, 13> kInternalLowerBodyKeypoints{
    KeypointId::LeftHip,
    KeypointId::RightHip,
    KeypointId::LeftKnee,
    KeypointId::RightKnee,
    KeypointId::LeftAnkle,
    KeypointId::RightAnkle,
    KeypointId::Pelvis,
    KeypointId::LeftBigToe,
    KeypointId::RightBigToe,
    KeypointId::LeftSmallToe,
    KeypointId::RightSmallToe,
    KeypointId::LeftHeel,
    KeypointId::RightHeel,
};

inline constexpr std::size_t KeypointIndex(KeypointId id) noexcept {
    return static_cast<std::size_t>(id);
}

inline constexpr bool InternalKeypointOrderIsCanonical() noexcept {
    for (std::size_t i = 0; i < kInternalKeypointOrder.size(); ++i) {
        if (KeypointIndex(kInternalKeypointOrder[i]) != i) {
            return false;
        }
    }
    return true;
}

inline bool IsFootKeypoint(KeypointId id) {
    return id == KeypointId::LeftBigToe ||
        id == KeypointId::RightBigToe ||
        id == KeypointId::LeftSmallToe ||
        id == KeypointId::RightSmallToe ||
        id == KeypointId::LeftHeel ||
        id == KeypointId::RightHeel;
}

inline bool IsLowerBodyKeypoint(KeypointId id) {
    return id == KeypointId::LeftHip ||
        id == KeypointId::RightHip ||
        id == KeypointId::LeftKnee ||
        id == KeypointId::RightKnee ||
        id == KeypointId::LeftAnkle ||
        id == KeypointId::RightAnkle ||
        id == KeypointId::Pelvis ||
        IsFootKeypoint(id);
}

// Backward-compatible aliases. Prefer the kInternal* names in new code.
inline constexpr auto& kHalpe26KeypointOrder = kInternalKeypointOrder;
inline constexpr auto& kHalpe26KeypointNames = kInternalKeypointNames;
inline constexpr auto& kHalpe26LowerBodyKeypoints = kInternalLowerBodyKeypoints;
inline constexpr bool Halpe26KeypointOrderIsCanonical() noexcept {
    return InternalKeypointOrderIsCanonical();
}

} // namespace bt
