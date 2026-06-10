#include "tracking/root_support.h"
#include "test_check.h"

namespace {

bt::Pose3f RootAt(float y) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{0.0f, y, 0.0f};
    return pose;
}

bt::Pose3f FootAt(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}

bt::FloorPlane Floor(float y) {
    bt::FloorPlane floor;
    floor.valid = true;
    floor.normal = bt::Vec3f{0.0f, 1.0f, 0.0f};
    floor.distance = y;
    return floor;
}

bt::FootSupportState ActiveFootSupport() {
    bt::FootSupportState foot;
    foot.type = bt::FootSupportType::FloorSupport;
    foot.phase = bt::FootSupportPhase::FlatPlant;
    foot.anchor.active = true;
    foot.anchor.confidence = 0.95f;
    return foot;
}

} // namespace

int main() {
    bt::RootSupportConfig config;
    config.release_seconds = 0.01;
    config.transition_hold_seconds = 0.01;

    bt::LowerBodyModel model;
    bt::LowerBodyState solved;
    solved.root = RootAt(0.9f);
    solved.left_foot = FootAt(-0.18f, 0.0f, 0.10f);
    solved.right_foot = FootAt(0.18f, 0.0f, 0.10f);
    solved.confidence = 1.0f;

    auto no_contact_kneel = bt::UpdateRootSupport(
        {},
        solved,
        bt::PostureMode::Kneeling,
        0.20,
        config);
    BT_CHECK(no_contact_kneel.root_support == bt::RootSupportType::None);
    BT_CHECK(!no_contact_kneel.root_anchor.active);

    bt::SupportManifoldState foot_contact;
    foot_contact.left_foot = ActiveFootSupport();
    auto foot_backed_kneel = bt::UpdateRootSupport(
        foot_contact,
        solved,
        bt::PostureMode::Kneeling,
        0.20,
        config);
    BT_CHECK(foot_backed_kneel.root_support == bt::RootSupportType::FeetSupported);
    BT_CHECK(foot_backed_kneel.root_anchor.active);

    bt::KneeContactEvidence no_knee_evidence;
    no_knee_evidence.left_usable = false;
    no_knee_evidence.right_usable = false;
    auto no_knee_anchor = bt::UpdateKneeContactSupport(
        {},
        solved,
        model,
        Floor(0.0f),
        bt::PostureMode::Kneeling,
        0.20,
        no_knee_evidence);
    BT_CHECK(!no_knee_anchor.left_knee_anchor.active);
    BT_CHECK(!no_knee_anchor.right_knee_anchor.active);

    bt::KneeContactEvidence knee_evidence;
    knee_evidence.left_usable = true;
    knee_evidence.left_confidence = 0.86f;
    auto knee_anchor = bt::UpdateKneeContactSupport(
        {},
        solved,
        model,
        Floor(0.0f),
        bt::PostureMode::Kneeling,
        0.20,
        knee_evidence);
    BT_CHECK(knee_anchor.left_knee_anchor.active);
    BT_CHECK(!knee_anchor.right_knee_anchor.active);

    auto knee_root = bt::UpdateRootSupport(
        knee_anchor,
        solved,
        bt::PostureMode::Kneeling,
        0.20,
        config);
    BT_CHECK(knee_root.root_support == bt::RootSupportType::KneeSupported);
    BT_CHECK(knee_root.root_anchor.active);

    knee_anchor.left_foot = ActiveFootSupport();
    auto mixed_root = bt::UpdateRootSupport(
        knee_anchor,
        solved,
        bt::PostureMode::Kneeling,
        0.20,
        config);
    BT_CHECK(mixed_root.root_support == bt::RootSupportType::MixedSupported);
    BT_CHECK(mixed_root.root_anchor.active);

    auto false_upright_knee = bt::UpdateKneeContactSupport(
        {},
        solved,
        model,
        Floor(0.0f),
        bt::PostureMode::UprightStanding,
        0.20,
        knee_evidence);
    BT_CHECK(!false_upright_knee.left_knee_anchor.active);

    return 0;
}
