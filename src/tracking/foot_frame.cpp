#include "tracking/foot_frame.h"

#include <algorithm>

namespace bt {
namespace {

const Keypoint3D& Joint(const LowerBodyJointSet& joints, KeypointId id) {
    return joints.joints[static_cast<std::size_t>(id)];
}

float ConfidenceOf(const Keypoint3D& kp) {
    return kp.present ? kp.confidence : 0.0f;
}

} // namespace

FootFrameEstimate InferFootFrame(
    BodySide side,
    const LowerBodyJointSet& joints,
    const FootSupportState& previous_support,
    const Pose3f& previous_foot_pose,
    const LowerBodyModel& model) {

    const bool left = side == BodySide::Left;
    const auto& ankle = Joint(joints, left ? KeypointId::LeftAnkle : KeypointId::RightAnkle);
    const auto& heel = Joint(joints, left ? KeypointId::LeftHeel : KeypointId::RightHeel);
    const auto& big_toe = Joint(joints, left ? KeypointId::LeftBigToe : KeypointId::RightBigToe);
    const auto& small_toe = Joint(joints, left ? KeypointId::LeftSmallToe : KeypointId::RightSmallToe);
    const auto& knee = Joint(joints, left ? KeypointId::LeftKnee : KeypointId::RightKnee);

    FootFrameEstimate out;
    out.foot_pose = previous_foot_pose;
    if (!ankle.present) {
        out.confidence = previous_support.anchor.active ? 0.25f * previous_support.anchor.confidence : 0.0f;
        out.valid = out.confidence > 0.0f;
        return out;
    }

    out.ankle = ankle.world;
    out.foot_pose.position = ankle.world;

    const bool has_big = big_toe.present && big_toe.confidence > 0.10f;
    const bool has_small = small_toe.present && small_toe.confidence > 0.10f;
    const bool has_heel = heel.present && heel.confidence > 0.10f;

    if ((has_big || has_small) && has_heel) {
        if (has_big && has_small) {
            out.toe = Scale(Add(big_toe.world, small_toe.world), 0.5f);
        } else {
            out.toe = has_big ? big_toe.world : small_toe.world;
        }
        out.heel = heel.world;
        out.forward_axis = NormalizeOr(Sub(out.toe, out.heel), out.forward_axis);
        out.sole_center = Scale(Add(out.toe, out.heel), 0.5f);
        out.lateral_axis = NormalizeOr(
            has_big && has_small ? Sub(small_toe.world, big_toe.world) : Cross(Vec3f{0.0f, 1.0f, 0.0f}, out.forward_axis),
            out.lateral_axis);
        out.up_axis = NormalizeOr(Cross(out.forward_axis, out.lateral_axis), out.up_axis);
        out.lateral_axis = NormalizeOr(Cross(out.up_axis, out.forward_axis), out.lateral_axis);
        out.used_toe_heel = true;
        out.confidence = std::min(1.0f, (ConfidenceOf(ankle) + ConfidenceOf(heel) + ConfidenceOf(big_toe) + ConfidenceOf(small_toe)) / 3.0f);
    } else {
        const float foot_len = left ? model.left_foot_length : model.right_foot_length;
        Vec3f shank_forward = Vec3f{0.0f, 0.0f, 1.0f};
        if (knee.present) {
            shank_forward = NormalizeOr(Sub(ankle.world, knee.world), shank_forward);
            shank_forward.y = 0.0f;
            shank_forward = NormalizeOr(shank_forward, Vec3f{0.0f, 0.0f, 1.0f});
        }
        if (previous_support.anchor.active) {
            out.foot_pose.orientation = previous_support.anchor.pose.orientation;
        }
        out.forward_axis = shank_forward;
        out.up_axis = Vec3f{0.0f, 1.0f, 0.0f};
        out.lateral_axis = NormalizeOr(Cross(out.up_axis, out.forward_axis), out.lateral_axis);

        // Foot anatomy convention for synthesized contacts:
        // - The ankle is treated as 30% of foot length forward from the heel.
        // - Heel = -0.30 * foot_length behind the ankle.
        // - Toe  = +0.70 * foot_length in front of the ankle.
        //
        // This is deliberately different from the simplified 50/50 contact
        // model used by contact_constraints.cpp, where the foot pose is already
        // expressed at the sole center. This path starts from an ankle-only
        // measurement, so it uses the anatomical ankle-to-heel/toe split rather
        // than pretending the ankle is the middle of the foot.
        out.heel = Add(ankle.world, Scale(out.forward_axis, -0.30f * foot_len));
        out.toe = Add(ankle.world, Scale(out.forward_axis, 0.70f * foot_len));
        out.sole_center = Scale(Add(out.heel, out.toe), 0.5f);
        out.confidence = 0.45f * ConfidenceOf(ankle) + (previous_support.anchor.active ? 0.20f * previous_support.anchor.confidence : 0.0f);
    }

    out.valid = out.confidence > 0.05f;
    out.foot_pose.position = out.sole_center;
    out.foot_pose.orientation = QuatFromBasis(out.lateral_axis, out.up_axis, out.forward_axis);
    return out;
}

} // namespace bt
