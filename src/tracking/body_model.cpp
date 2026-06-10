#include "tracking/body_model.h"

#include "inference/keypoint_contract.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace bt {

LowerBodyModel MakeLowerBodyModel(const BodyCalibration& c) {
    LowerBodyModel model;
    model.pelvis_width = c.pelvis_width;
    model.left_femur = c.left_femur;
    model.right_femur = c.right_femur;
    model.left_tibia = c.left_tibia;
    model.right_tibia = c.right_tibia;
    model.left_foot_length = c.left_foot_length;
    model.right_foot_length = c.right_foot_length;
    model.standing_hmd_to_pelvis = c.standing_hmd_to_pelvis;
    model.seated_hmd_to_pelvis = c.seated_hmd_to_pelvis;
    model.reclined_hmd_to_pelvis = c.reclined_hmd_to_pelvis;
    return model;
}

std::string BuildBodyModelSummary(const LowerBodyModel& model) {
    std::ostringstream oss;
    oss << "body_model pelvis_width=" << model.pelvis_width
        << " femur=(" << model.left_femur << "," << model.right_femur << ")"
        << " tibia=(" << model.left_tibia << "," << model.right_tibia << ")"
        << " foot=(" << model.left_foot_length << "," << model.right_foot_length << ")";
    return oss.str();
}

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kFootCenterForwardFrac = 0.20f;
constexpr float kFootCenterBelowAnkleM = 0.035f;
constexpr float kToeHalfSpanM = 0.035f;

struct BodyBasis {
    Vec3f right{1.0f, 0.0f, 0.0f};
    Vec3f up{0.0f, 1.0f, 0.0f};
    Vec3f forward{0.0f, 0.0f, 1.0f};
};

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

float Clamp(float v, float lo, float hi) {
    if (!std::isfinite(v)) {
        return lo;
    }
    return std::max(lo, std::min(hi, v));
}

BodyBasis BasisFromRoot(const Pose3f& root) {
    BodyBasis basis;
    const Quatf q = Normalize(root.orientation);
    basis.right = NormalizeOr(Rotate(q, Vec3f{1.0f, 0.0f, 0.0f}), Vec3f{1.0f, 0.0f, 0.0f});
    basis.up = NormalizeOr(Rotate(q, Vec3f{0.0f, 1.0f, 0.0f}), Vec3f{0.0f, 1.0f, 0.0f});
    basis.forward = NormalizeOr(Rotate(q, Vec3f{0.0f, 0.0f, 1.0f}), Vec3f{0.0f, 0.0f, 1.0f});

    // Re-orthogonalize after numeric drift. Unlike the old basis, this preserves
    // pelvis roll/pitch when hip/knee evidence has produced it; yaw-only HMD
    // input still behaves exactly like the previous path.
    basis.forward = NormalizeOr(Cross(basis.right, basis.up), basis.forward);
    basis.up = NormalizeOr(Cross(basis.forward, basis.right), basis.up);
    return basis;
}

float CoordInBasis(const Vec3f& v, const Vec3f& axis) {
    return Dot(v, axis);
}

Vec3f FootForwardAxis(const Pose3f& foot_pose, const BodyBasis& fallback_basis) {
    Vec3f forward = Rotate(Normalize(foot_pose.orientation), Vec3f{0.0f, 0.0f, 1.0f});
    if (Length(forward) < 1e-5f || !IsFinite(forward)) {
        return fallback_basis.forward;
    }
    return NormalizeOr(forward, fallback_basis.forward);
}

Vec3f FootUpAxis(const Pose3f& foot_pose) {
    return NormalizeOr(Rotate(Normalize(foot_pose.orientation), Vec3f{0.0f, 1.0f, 0.0f}), Vec3f{0.0f, 1.0f, 0.0f});
}

Vec3f FootRightAxis(const Pose3f& foot_pose, const BodyBasis& fallback_basis) {
    return NormalizeOr(Rotate(Normalize(foot_pose.orientation), Vec3f{1.0f, 0.0f, 0.0f}), fallback_basis.right);
}

Vec3f AnkleFromFootPose(const Pose3f& foot_pose, const LowerBodyModel& model, bool left, const BodyBasis& basis) {
    const float foot_length = left ? model.left_foot_length : model.right_foot_length;
    const Vec3f forward = FootForwardAxis(foot_pose, basis);
    const Vec3f up = FootUpAxis(foot_pose);
    return Add(
        Sub(foot_pose.position, Scale(forward, kFootCenterForwardFrac * foot_length)),
        Scale(up, kFootCenterBelowAnkleM));
}

void Put(LowerBodyJointSet& set, KeypointId id, Vec3f p, float confidence) {
    auto& joint = set.joints[static_cast<std::size_t>(id)];
    joint.world = p;
    joint.confidence = confidence;
    joint.present = confidence > 0.0f;
}

float SegmentLengthConsistency(float measured, float expected) {
    if (!std::isfinite(measured) || !std::isfinite(expected) || expected <= 1e-4f) {
        return 0.0f;
    }
    const float relative_error = std::abs(measured - expected) / expected;
    if (relative_error <= 0.12f) {
        return 1.0f;
    }
    if (relative_error >= 0.45f) {
        return 0.0f;
    }
    return 1.0f - (relative_error - 0.12f) / (0.45f - 0.12f);
}

float SegmentSeedGain(const Keypoint3D& a, const Keypoint3D& b, float expected_length, float seed_weight) {
    if (!a.present || !b.present) {
        return 0.0f;
    }
    return Clamp01(seed_weight * SegmentLengthConsistency(Distance(a.world, b.world), expected_length));
}

float WeakSinglePointGain(float seed_weight) {
    return Clamp01(seed_weight * 0.35f);
}

float EstimateHipAbductionFromVector(const BodyBasis& basis, const Vec3f& hip_to_distal, bool left) {
    const float side = left ? -1.0f : 1.0f;
    const float lateral = side * CoordInBasis(hip_to_distal, basis.right);
    const float vertical = CoordInBasis(hip_to_distal, basis.up);
    const float forward = CoordInBasis(hip_to_distal, basis.forward);
    const float planar = std::sqrt(vertical * vertical + forward * forward);
    return Clamp(std::atan2(lateral, std::max(1e-5f, planar)), -0.95f, 0.95f);
}

float EstimateHipFlexionFromVector(const BodyBasis& basis, const Vec3f& hip_to_distal) {
    const float down = -CoordInBasis(hip_to_distal, basis.up);
    const float forward = CoordInBasis(hip_to_distal, basis.forward);
    return std::atan2(forward, std::max(1e-5f, down));
}

float KneeFlexionFromSegments(const Vec3f& femur_vec, const Vec3f& tibia_vec) {
    const Vec3f a = NormalizeOr(femur_vec, Vec3f{0.0f, -1.0f, 0.0f});
    const Vec3f b = NormalizeOr(tibia_vec, Vec3f{0.0f, -1.0f, 0.0f});
    return Clamp(std::acos(Clamp(Dot(a, b), -1.0f, 1.0f)), 0.0f, 2.75f);
}

void EstimateFootAngles(
    const BodyBasis& basis,
    const Pose3f& foot_pose,
    bool left,
    float* out_pitch,
    float* out_roll,
    float* out_yaw) {

    const Vec3f forward = FootForwardAxis(foot_pose, basis);
    const Vec3f up = FootUpAxis(foot_pose);
    const float horizontal = std::sqrt(
        CoordInBasis(forward, basis.right) * CoordInBasis(forward, basis.right) +
        CoordInBasis(forward, basis.forward) * CoordInBasis(forward, basis.forward));
    if (out_pitch) {
        *out_pitch = std::atan2(CoordInBasis(forward, basis.up), std::max(1e-5f, horizontal));
    }
    if (out_roll) {
        const float side = left ? -1.0f : 1.0f;
        *out_roll = std::atan2(side * CoordInBasis(up, basis.right), std::max(1e-5f, CoordInBasis(up, basis.up)));
    }
    if (out_yaw) {
        *out_yaw = std::atan2(CoordInBasis(forward, basis.right), std::max(1e-5f, CoordInBasis(forward, basis.forward)));
    }
}

Vec3f ProjectOntoPlane(Vec3f v, const Vec3f& normal, const Vec3f& fallback) {
    v = Sub(v, Scale(normal, Dot(v, normal)));
    return NormalizeOr(v, fallback);
}

float ClampReach(float reach, float femur, float tibia) {
    const float max_reach = std::max(1e-4f, femur + tibia - 1e-3f);
    const float min_reach = std::max(1e-4f, std::abs(femur - tibia) + 1e-3f);
    return Clamp(reach, min_reach, max_reach);
}

Vec3f KneeBendHint(const BodyBasis& basis, const LowerBodyState& state, bool left, const Vec3f& hip_to_ankle_dir) {
    const float bend_hint = left ? state.left_hip_flexion : state.right_hip_flexion;

    // The foot target already defines the lateral hip-to-ankle direction. The
    // knee plane should not add a second sideways tibia cheat. It only chooses
    // which side of the constrained two-bone triangle the knee bends toward.
    const Vec3f sagittal_hint = NormalizeOr(
        Add(Scale(basis.forward, std::sin(bend_hint)), Scale(basis.up, -std::cos(bend_hint))),
        Scale(basis.up, -1.0f));
    return ProjectOntoPlane(
        sagittal_hint,
        hip_to_ankle_dir,
        ProjectOntoPlane(basis.forward, hip_to_ankle_dir, Scale(basis.up, -1.0f)));
}

struct TwoBoneLegSolution {
    Vec3f ankle{};
    Vec3f knee{};
    bool valid = false;
    bool reach_clamped = false;
};

TwoBoneLegSolution SolveTwoBoneLeg(
    const BodyBasis& basis,
    const LowerBodyState& state,
    const Vec3f& hip,
    const Vec3f& desired_ankle,
    float femur,
    float tibia,
    bool left) {

    TwoBoneLegSolution out;
    if (femur <= 1e-4f || tibia <= 1e-4f) {
        return out;
    }

    Vec3f hip_to_ankle = Sub(desired_ankle, hip);
    const float desired_reach = Length(hip_to_ankle);
    const Vec3f dir = NormalizeOr(hip_to_ankle, Scale(basis.up, -1.0f));
    const float reach = ClampReach(desired_reach, femur, tibia);
    out.reach_clamped = std::abs(reach - desired_reach) > 1e-4f;
    out.ankle = Add(hip, Scale(dir, reach));

    const float along = Clamp((femur * femur - tibia * tibia + reach * reach) / (2.0f * reach), 0.0f, femur);
    const float height_sq = std::max(0.0f, femur * femur - along * along);
    const float height = std::sqrt(height_sq);
    const Vec3f bend = KneeBendHint(basis, state, left, dir);
    out.knee = Add(Add(hip, Scale(dir, along)), Scale(bend, height));
    out.valid = true;
    return out;
}

Quatf PelvisOrientationFromLegSeeds(
    const Keypoint3D& left_hip,
    const Keypoint3D& right_hip,
    const Keypoint3D& left_knee,
    const Keypoint3D& right_knee,
    const Quatf& fallback) {

    if (!left_hip.present || !right_hip.present) {
        return fallback;
    }

    Vec3f right = NormalizeOr(Sub(right_hip.world, left_hip.world), Rotate(fallback, Vec3f{1.0f, 0.0f, 0.0f}));
    Vec3f down{};
    float weight = 0.0f;
    if (left_knee.present) {
        down = Add(down, NormalizeOr(Sub(left_knee.world, left_hip.world), Vec3f{0.0f, -1.0f, 0.0f}));
        weight += 1.0f;
    }
    if (right_knee.present) {
        down = Add(down, NormalizeOr(Sub(right_knee.world, right_hip.world), Vec3f{0.0f, -1.0f, 0.0f}));
        weight += 1.0f;
    }

    if (weight <= 0.0f) {
        Vec3f fallback_up = ProjectOntoPlane(Rotate(fallback, Vec3f{0.0f, 1.0f, 0.0f}), right, Vec3f{0.0f, 1.0f, 0.0f});
        Vec3f forward = NormalizeOr(Cross(right, fallback_up), Rotate(fallback, Vec3f{0.0f, 0.0f, 1.0f}));
        fallback_up = NormalizeOr(Cross(forward, right), fallback_up);
        return QuatFromBasis(right, fallback_up, forward);
    }

    down = NormalizeOr(Scale(down, 1.0f / weight), Vec3f{0.0f, -1.0f, 0.0f});
    Vec3f up = ProjectOntoPlane(Scale(down, -1.0f), right, Rotate(fallback, Vec3f{0.0f, 1.0f, 0.0f}));
    Vec3f forward = NormalizeOr(Cross(right, up), Rotate(fallback, Vec3f{0.0f, 0.0f, 1.0f}));
    up = NormalizeOr(Cross(forward, right), up);
    return QuatFromBasis(right, up, forward);
}

} // namespace

Pose3f FootPoseFromAnkleTarget(
    const Vec3f& ankle,
    const Pose3f& previous_foot_pose,
    const LowerBodyModel& model,
    bool left) {

    const float foot_length = left ? model.left_foot_length : model.right_foot_length;
    const Quatf orientation = Normalize(previous_foot_pose.orientation);
    const Vec3f forward = NormalizeOr(Rotate(orientation, Vec3f{0.0f, 0.0f, 1.0f}), Vec3f{0.0f, 0.0f, 1.0f});
    const Vec3f up = NormalizeOr(Rotate(orientation, Vec3f{0.0f, 1.0f, 0.0f}), Vec3f{0.0f, 1.0f, 0.0f});

    Pose3f out = previous_foot_pose;
    out.orientation = orientation;
    out.position = Add(
        Add(ankle, Scale(forward, kFootCenterForwardFrac * foot_length)),
        Scale(up, -kFootCenterBelowAnkleM));
    return out;
}

void SolveLeg3DFromFootTarget(LowerBodyState& state, const LowerBodyModel& model, bool left) {
    const float femur = left ? model.left_femur : model.right_femur;
    const float tibia = left ? model.left_tibia : model.right_tibia;
    if (femur <= 1e-4f || tibia <= 1e-4f) {
        return;
    }

    const BodyBasis basis = BasisFromRoot(state.root);
    const float side = left ? -1.0f : 1.0f;
    const Vec3f hip = Add(state.root.position, Scale(basis.right, side * 0.5f * model.pelvis_width));
    Pose3f foot_pose = left ? state.left_foot : state.right_foot;
    const Vec3f desired_ankle = AnkleFromFootPose(foot_pose, model, left, basis);
    const TwoBoneLegSolution solved = SolveTwoBoneLeg(basis, state, hip, desired_ankle, femur, tibia, left);
    if (left) {
        state.left_leg_reach_clamped = solved.reach_clamped;
    } else {
        state.right_leg_reach_clamped = solved.reach_clamped;
    }
    if (!solved.valid) {
        return;
    }

    if (solved.reach_clamped) {
        foot_pose.position = Add(foot_pose.position, Sub(solved.ankle, desired_ankle));
        if (left) {
            state.left_foot.position = foot_pose.position;
        } else {
            state.right_foot.position = foot_pose.position;
        }
    }

    const Vec3f femur_vec = Sub(solved.knee, hip);
    const Vec3f tibia_vec = Sub(solved.ankle, solved.knee);

    // The two-bone solve may choose a mostly sagittal knee plane when direct knee
    // evidence is absent. Hip flexion/abduction should still represent the
    // reachable ankle target, otherwise a laterally placed foot can solve to
    // correct joint positions while exporting zero hip abduction.
    const Vec3f hip_to_ankle = Sub(solved.ankle, hip);
    const float hip_flexion = EstimateHipFlexionFromVector(basis, hip_to_ankle);
    const float hip_abduction = EstimateHipAbductionFromVector(basis, hip_to_ankle, left);
    const float knee_flexion = KneeFlexionFromSegments(femur_vec, tibia_vec);

    float ankle_pitch = 0.0f;
    float ankle_roll = 0.0f;
    float ankle_yaw = 0.0f;
    EstimateFootAngles(basis, foot_pose, left, &ankle_pitch, &ankle_roll, &ankle_yaw);

    if (left) {
        state.left_hip_flexion = hip_flexion;
        state.left_hip_abduction = hip_abduction;
        state.left_knee_flexion = knee_flexion;
        state.left_ankle_pitch = ankle_pitch;
        state.left_ankle_roll = ankle_roll;
        state.left_ankle_yaw = ankle_yaw;
    } else {
        state.right_hip_flexion = hip_flexion;
        state.right_hip_abduction = hip_abduction;
        state.right_knee_flexion = knee_flexion;
        state.right_ankle_pitch = ankle_pitch;
        state.right_ankle_roll = ankle_roll;
        state.right_ankle_yaw = ankle_yaw;
    }
}

void SolveSagittalLegFromFootTarget(LowerBodyState& state, const LowerBodyModel& model, bool left) {
    SolveLeg3DFromFootTarget(state, model, left);
}

LowerBodyJointSet PredictLowerBodyJoints(const LowerBodyState& state, const LowerBodyModel& model) {
    LowerBodyJointSet set;

    const BodyBasis basis = BasisFromRoot(state.root);
    const Vec3f pelvis = state.root.position;
    const Vec3f left_hip = Add(pelvis, Scale(basis.right, -0.5f * model.pelvis_width));
    const Vec3f right_hip = Add(pelvis, Scale(basis.right, 0.5f * model.pelvis_width));

    const Vec3f desired_left_ankle = AnkleFromFootPose(state.left_foot, model, true, basis);
    const Vec3f desired_right_ankle = AnkleFromFootPose(state.right_foot, model, false, basis);
    const TwoBoneLegSolution left_leg = SolveTwoBoneLeg(
        basis, state, left_hip, desired_left_ankle, model.left_femur, model.left_tibia, true);
    const TwoBoneLegSolution right_leg = SolveTwoBoneLeg(
        basis, state, right_hip, desired_right_ankle, model.right_femur, model.right_tibia, false);

    const Vec3f left_knee = left_leg.valid ? left_leg.knee : Add(left_hip, Scale(basis.up, -model.left_femur));
    const Vec3f right_knee = right_leg.valid ? right_leg.knee : Add(right_hip, Scale(basis.up, -model.right_femur));
    const Vec3f left_ankle = left_leg.valid ? left_leg.ankle : desired_left_ankle;
    const Vec3f right_ankle = right_leg.valid ? right_leg.ankle : desired_right_ankle;

    // If the requested foot target was reachable, preserve the tracker foot pose.
    // If not, project the foot-center along with the clamped ankle so predicted
    // joints do not advertise impossible segment lengths.
    Pose3f left_foot_pose = state.left_foot;
    Pose3f right_foot_pose = state.right_foot;
    if (left_leg.valid && left_leg.reach_clamped) {
        left_foot_pose.position = Add(left_foot_pose.position, Sub(left_ankle, desired_left_ankle));
    }
    if (right_leg.valid && right_leg.reach_clamped) {
        right_foot_pose.position = Add(right_foot_pose.position, Sub(right_ankle, desired_right_ankle));
    }

    const Vec3f left_forward = FootForwardAxis(left_foot_pose, basis);
    const Vec3f left_up = FootUpAxis(left_foot_pose);
    const Vec3f left_right = FootRightAxis(left_foot_pose, basis);
    const Vec3f right_forward = FootForwardAxis(right_foot_pose, basis);
    const Vec3f right_up = FootUpAxis(right_foot_pose);
    const Vec3f right_right = FootRightAxis(right_foot_pose, basis);

    const Vec3f left_toe = Add(Add(left_foot_pose.position, Scale(left_forward, 0.50f * model.left_foot_length)), Scale(left_up, 0.005f));
    const Vec3f left_heel = Add(Add(left_foot_pose.position, Scale(left_forward, -0.50f * model.left_foot_length)), Scale(left_up, -0.005f));
    const Vec3f right_toe = Add(Add(right_foot_pose.position, Scale(right_forward, 0.50f * model.right_foot_length)), Scale(right_up, 0.005f));
    const Vec3f right_heel = Add(Add(right_foot_pose.position, Scale(right_forward, -0.50f * model.right_foot_length)), Scale(right_up, -0.005f));

    Put(set, KeypointId::Pelvis, pelvis, state.confidence);
    Put(set, KeypointId::LeftHip, left_hip, state.confidence);
    Put(set, KeypointId::RightHip, right_hip, state.confidence);
    Put(set, KeypointId::LeftKnee, left_knee, state.confidence);
    Put(set, KeypointId::RightKnee, right_knee, state.confidence);
    Put(set, KeypointId::LeftAnkle, left_ankle, state.confidence);
    Put(set, KeypointId::RightAnkle, right_ankle, state.confidence);
    Put(set, KeypointId::LeftBigToe, left_toe, state.confidence);
    Put(set, KeypointId::LeftSmallToe, Add(Add(left_toe, Scale(left_right, -kToeHalfSpanM)), Scale(left_forward, -0.01f)), state.confidence);
    Put(set, KeypointId::LeftHeel, left_heel, state.confidence);
    Put(set, KeypointId::RightBigToe, right_toe, state.confidence);
    Put(set, KeypointId::RightSmallToe, Add(Add(right_toe, Scale(right_right, kToeHalfSpanM)), Scale(right_forward, -0.01f)), state.confidence);
    Put(set, KeypointId::RightHeel, right_heel, state.confidence);

    return set;
}

LowerBodyState EstimateStateFromJointSeeds(
    const LowerBodyState& predicted,
    const LowerBodyModel& model,
    const LowerBodyJointSet& seeds,
    float seed_weight) {

    LowerBodyState out = predicted;
    const auto read = [&](KeypointId id) -> const Keypoint3D& {
        return seeds.joints[static_cast<std::size_t>(id)];
    };

    const auto& lh = read(KeypointId::LeftHip);
    const auto& rh = read(KeypointId::RightHip);
    if (lh.present && rh.present) {
        const float pelvis_gain = SegmentSeedGain(lh, rh, model.pelvis_width, seed_weight);
        const Vec3f pelvis = Scale(Add(lh.world, rh.world), 0.5f);
        out.root.position = Lerp(out.root.position, pelvis, pelvis_gain);
        out.root.orientation = Slerp(
            out.root.orientation,
            PelvisOrientationFromLegSeeds(lh, rh, read(KeypointId::LeftKnee), read(KeypointId::RightKnee), out.root.orientation),
            pelvis_gain);
    } else if (read(KeypointId::Pelvis).present) {
        out.root.position = Lerp(out.root.position, read(KeypointId::Pelvis).world, WeakSinglePointGain(seed_weight));
    }

    const BodyBasis basis = BasisFromRoot(out.root);

    const auto solve_leg = [&](bool left) {
        const KeypointId hip_id = left ? KeypointId::LeftHip : KeypointId::RightHip;
        const KeypointId knee_id = left ? KeypointId::LeftKnee : KeypointId::RightKnee;
        const KeypointId ankle_id = left ? KeypointId::LeftAnkle : KeypointId::RightAnkle;
        const KeypointId toe_id = left ? KeypointId::LeftBigToe : KeypointId::RightBigToe;
        const KeypointId small_toe_id = left ? KeypointId::LeftSmallToe : KeypointId::RightSmallToe;
        const KeypointId heel_id = left ? KeypointId::LeftHeel : KeypointId::RightHeel;
        const float femur = left ? model.left_femur : model.right_femur;
        const float tibia = left ? model.left_tibia : model.right_tibia;
        const float foot_length = left ? model.left_foot_length : model.right_foot_length;
        const auto& hip = read(hip_id);
        const auto& knee = read(knee_id);
        const auto& ankle = read(ankle_id);
        const auto& toe = read(toe_id);
        const auto& small_toe = read(small_toe_id);
        const auto& heel = read(heel_id);

        const float femur_gain = SegmentSeedGain(hip, knee, femur, seed_weight);
        const float tibia_gain = SegmentSeedGain(knee, ankle, tibia, seed_weight);
        const float leg_chain_gain = (femur_gain > 0.0f && tibia_gain > 0.0f)
            ? std::min(femur_gain, tibia_gain)
            : std::max(femur_gain, tibia_gain) * 0.45f;

        if (hip.present && knee.present && femur_gain > 0.0f) {
            const Vec3f hk = Sub(knee.world, hip.world);
            const float hip_flex = EstimateHipFlexionFromVector(basis, hk);
            const float hip_abd = EstimateHipAbductionFromVector(basis, hk, left);
            if (left) {
                out.left_hip_flexion = Lerp(out.left_hip_flexion, hip_flex, femur_gain);
                out.left_hip_abduction = Lerp(out.left_hip_abduction, hip_abd, femur_gain);
            } else {
                out.right_hip_flexion = Lerp(out.right_hip_flexion, hip_flex, femur_gain);
                out.right_hip_abduction = Lerp(out.right_hip_abduction, hip_abd, femur_gain);
            }
        }

        if (hip.present && knee.present && ankle.present && leg_chain_gain > 0.0f) {
            const Vec3f hk = Sub(knee.world, hip.world);
            const Vec3f ka = Sub(ankle.world, knee.world);
            const float knee_flex = KneeFlexionFromSegments(hk, ka);
            if (left) {
                out.left_knee_flexion = Lerp(out.left_knee_flexion, knee_flex, leg_chain_gain);
            } else {
                out.right_knee_flexion = Lerp(out.right_knee_flexion, knee_flex, leg_chain_gain);
            }
        }

        if (ankle.present) {
            const float ankle_gain = leg_chain_gain > 0.0f ? leg_chain_gain : WeakSinglePointGain(seed_weight);
            Pose3f foot_pose = FootPoseFromAnkleTarget(ankle.world, left ? out.left_foot : out.right_foot, model, left);
            if (left) {
                out.left_foot.position = Lerp(out.left_foot.position, foot_pose.position, ankle_gain);
            } else {
                out.right_foot.position = Lerp(out.right_foot.position, foot_pose.position, ankle_gain);
            }
        }

        if (toe.present && heel.present) {
            const float foot_gain = SegmentSeedGain(toe, heel, foot_length, seed_weight);
            if (foot_gain > 0.0f) {
                const Vec3f toe_center = (small_toe.present && small_toe.confidence > 0.05f)
                    ? Scale(Add(toe.world, small_toe.world), 0.5f)
                    : toe.world;
                const Vec3f heel_to_toe = NormalizeOr(Sub(toe_center, heel.world), basis.forward);
                Vec3f lateral = small_toe.present
                    ? NormalizeOr(Sub(small_toe.world, toe.world), left ? Scale(basis.right, -1.0f) : basis.right)
                    : NormalizeOr(Cross(Vec3f{0.0f, 1.0f, 0.0f}, heel_to_toe), basis.right);
                Vec3f up = NormalizeOr(Cross(heel_to_toe, lateral), basis.up);
                lateral = NormalizeOr(Cross(up, heel_to_toe), lateral);
                Pose3f foot_pose;
                foot_pose.position = Scale(Add(toe_center, heel.world), 0.5f);
                foot_pose.orientation = QuatFromBasis(lateral, up, heel_to_toe);
                float pitch = 0.0f;
                float roll = 0.0f;
                float yaw = 0.0f;
                EstimateFootAngles(basis, foot_pose, left, &pitch, &roll, &yaw);
                if (left) {
                    out.left_foot.position = Lerp(out.left_foot.position, foot_pose.position, foot_gain);
                    out.left_foot.orientation = Slerp(out.left_foot.orientation, foot_pose.orientation, foot_gain);
                    out.left_ankle_pitch = Lerp(out.left_ankle_pitch, pitch, foot_gain);
                    out.left_ankle_roll = Lerp(out.left_ankle_roll, roll, foot_gain);
                    out.left_ankle_yaw = Lerp(out.left_ankle_yaw, yaw, foot_gain);
                } else {
                    out.right_foot.position = Lerp(out.right_foot.position, foot_pose.position, foot_gain);
                    out.right_foot.orientation = Slerp(out.right_foot.orientation, foot_pose.orientation, foot_gain);
                    out.right_ankle_pitch = Lerp(out.right_ankle_pitch, pitch, foot_gain);
                    out.right_ankle_roll = Lerp(out.right_ankle_roll, roll, foot_gain);
                    out.right_ankle_yaw = Lerp(out.right_ankle_yaw, yaw, foot_gain);
                }
            }
        }
    };

    solve_leg(true);
    solve_leg(false);
    out.confidence = std::max(out.confidence, seed_weight);
    return out;
}

} // namespace bt
