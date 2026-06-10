#include "tracking/foot_support.h"
#include "tracking/contact_constraints.h"
#include "test_check.h"

#include <cmath>

namespace {

bt::Quatf Pitch(float radians) {
    const float half = 0.5f * radians;
    return bt::Quatf{std::sin(half), 0.0f, 0.0f, std::cos(half)};
}

bt::Pose3f Foot(float x, float y, float z, bt::Quatf orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f}) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = orientation;
    return pose;
}

bt::FloorPlane Floor(float y) {
    bt::FloorPlane floor;
    floor.valid = true;
    floor.normal = bt::Vec3f{0.0f, 1.0f, 0.0f};
    floor.distance = y;
    return floor;
}

} // namespace

int main() {
    bt::FootSupportConfig config;
    config.lock_dwell_seconds = 0.10;
    config.floor_height_epsilon = 0.05f;
    config.swing_motion_epsilon = 0.25f;

    bt::FootSupportEvidence good_evidence;
    good_evidence.confidence = 0.90f;
    good_evidence.usable = true;

    const auto floor = Floor(1.25f);
    const auto foot = Foot(0.2f, 1.27f, -0.1f);

    bt::FootSupportState support;
    support = bt::UpdateFootSupport(
        support,
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &good_evidence);

    BT_CHECK(support.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(support.phase == bt::FootSupportPhase::ContactCandidate);
    BT_CHECK(support.anchor.active);
    BT_CHECK_NEAR(support.anchor.pose.position.y, 1.25, 1e-5);

    support = bt::UpdateFootSupport(
        support,
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        0.20,
        config,
        &good_evidence);

    BT_CHECK(support.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(support.phase == bt::FootSupportPhase::FlatPlant);

    auto toe_support = support;
    const auto toe_pose = Foot(0.2f, 1.27f, -0.1f, Pitch(0.50f));
    toe_support = bt::UpdateFootSupport(
        toe_support,
        toe_pose,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &good_evidence);
    BT_CHECK(toe_support.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(toe_support.phase == bt::FootSupportPhase::ToePivot);
    BT_CHECK(toe_support.toe_anchor.active);

    auto heel_support = support;
    const auto heel_pose = Foot(0.2f, 1.27f, -0.1f, Pitch(-0.30f));
    heel_support = bt::UpdateFootSupport(
        heel_support,
        heel_pose,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &good_evidence);
    BT_CHECK(heel_support.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(heel_support.phase == bt::FootSupportPhase::HeelLock);
    BT_CHECK(heel_support.heel_anchor.active);

    const auto high_foot = Foot(0.2f, 1.50f, -0.1f);
    auto high_support = bt::UpdateFootSupport(
        {},
        high_foot,
        high_foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config);
    BT_CHECK(high_support.type == bt::FootSupportType::None);
    BT_CHECK(!high_support.anchor.active);

    bt::FloorPlane invalid_floor;
    const auto no_floor_support = bt::UpdateFootSupport(
        {},
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        invalid_floor,
        1.0 / 60.0,
        config);
    BT_CHECK(no_floor_support.type == bt::FootSupportType::None);


    bt::FootSupportEvidence weak_evidence;
    weak_evidence.confidence = 0.10f;
    weak_evidence.usable = true;
    auto weak_support = bt::UpdateFootSupport(
        {},
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        0.20,
        config,
        &weak_evidence);
    BT_CHECK(weak_support.type == bt::FootSupportType::None);
    BT_CHECK(weak_support.phase != bt::FootSupportPhase::FlatPlant);
    BT_CHECK(weak_support.phase != bt::FootSupportPhase::HeelLock);
    BT_CHECK(weak_support.phase != bt::FootSupportPhase::ToePivot);

    auto weak_release = support;
    weak_release = bt::UpdateFootSupport(
        weak_release,
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &weak_evidence);
    BT_CHECK(weak_release.phase == bt::FootSupportPhase::ReleasePending);
    BT_CHECK(weak_release.phase != bt::FootSupportPhase::FlatPlant);
    weak_release = bt::UpdateFootSupport(
        weak_release,
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        config.release_seconds,
        config,
        &weak_evidence);
    BT_CHECK(weak_release.type == bt::FootSupportType::None);
    BT_CHECK(weak_release.phase == bt::FootSupportPhase::Swing);
    BT_CHECK(!weak_release.anchor.active);

    const auto rest_pose = Foot(0.2f, 1.65f, -0.1f);

    bt::FootSupportEvidence missing_evidence;
    missing_evidence.confidence = 0.0f;
    missing_evidence.usable = false;
    auto missing_rest = bt::UpdateFootSupport(
        {},
        rest_pose,
        rest_pose,
        bt::PostureMode::SeatedSupported,
        floor,
        0.20,
        config,
        &missing_evidence);
    BT_CHECK(missing_rest.type == bt::FootSupportType::None);
    BT_CHECK(!missing_rest.anchor.active);

    auto weak_rest = bt::UpdateFootSupport(
        {},
        rest_pose,
        rest_pose,
        bt::PostureMode::SeatedSupported,
        floor,
        0.20,
        config,
        &weak_evidence);
    BT_CHECK(weak_rest.type == bt::FootSupportType::None);
    BT_CHECK(weak_rest.phase != bt::FootSupportPhase::RestLock);

    auto unknown_rest = bt::UpdateFootSupport(
        {},
        rest_pose,
        rest_pose,
        bt::PostureMode::UnknownFree,
        floor,
        0.20,
        config,
        &good_evidence);
    BT_CHECK(unknown_rest.type == bt::FootSupportType::None);

    auto good_rest = bt::UpdateFootSupport(
        {},
        rest_pose,
        rest_pose,
        bt::PostureMode::SeatedSupported,
        floor,
        1.0 / 60.0,
        config,
        &good_evidence);
    BT_CHECK(good_rest.type == bt::FootSupportType::RestSupport);
    BT_CHECK(good_rest.phase == bt::FootSupportPhase::RestCandidate);
    good_rest = bt::UpdateFootSupport(
        good_rest,
        rest_pose,
        rest_pose,
        bt::PostureMode::SeatedSupported,
        floor,
        0.20,
        config,
        &good_evidence);
    BT_CHECK(good_rest.type == bt::FootSupportType::RestSupport);
    BT_CHECK(good_rest.phase == bt::FootSupportPhase::RestLock);
    BT_CHECK(!good_rest.heel_anchor.active);
    BT_CHECK(!good_rest.toe_anchor.active);
    BT_CHECK(!bt::FootSupportHasContactConstraint(good_rest));
    BT_CHECK(!bt::FootSupportIsFullPlant(good_rest));

    auto no_evidence_floor = bt::UpdateFootSupport(
        {},
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        0.20,
        config);
    BT_CHECK(no_evidence_floor.type == bt::FootSupportType::None);
    BT_CHECK(!no_evidence_floor.anchor.active);

    auto good_toe_support = support;
    good_toe_support = bt::UpdateFootSupport(
        good_toe_support,
        toe_pose,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &good_evidence);
    BT_CHECK(good_toe_support.phase == bt::FootSupportPhase::ToePivot);
    BT_CHECK(good_toe_support.anchor.confidence <= good_evidence.confidence + 1e-5f);


    bt::FootSupportEvidence heel_only_evidence;
    heel_only_evidence.confidence = 0.88f;
    heel_only_evidence.heel_confidence = 0.88f;
    heel_only_evidence.toe_confidence = 0.02f;
    heel_only_evidence.heel_usable = true;
    heel_only_evidence.toe_usable = true;
    heel_only_evidence.usable = true;
    auto heel_only = bt::UpdateFootSupport(
        {},
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        0.20,
        config,
        &heel_only_evidence);
    BT_CHECK(heel_only.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(heel_only.phase == bt::FootSupportPhase::HeelLock);
    BT_CHECK(heel_only.contact_load == bt::FootContactLoad::HeelOnly);
    BT_CHECK(heel_only.heel_anchor.active);
    BT_CHECK(!heel_only.toe_anchor.active);

    bt::FootSupportEvidence toe_only_evidence = heel_only_evidence;
    toe_only_evidence.heel_confidence = 0.02f;
    toe_only_evidence.toe_confidence = 0.88f;
    auto toe_only = bt::UpdateFootSupport(
        {},
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        0.20,
        config,
        &toe_only_evidence);
    BT_CHECK(toe_only.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(toe_only.phase == bt::FootSupportPhase::ToePivot);
    BT_CHECK(toe_only.contact_load == bt::FootContactLoad::ToeOnly);
    BT_CHECK(!toe_only.heel_anchor.active);
    BT_CHECK(toe_only.toe_anchor.active);

    const auto backward_heel_pose = Foot(0.26f, 1.27f, -0.1f);
    auto backward_step = bt::UpdateFootSupport(
        toe_only,
        backward_heel_pose,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &heel_only_evidence);
    BT_CHECK(backward_step.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(backward_step.phase == bt::FootSupportPhase::HeelLock);
    BT_CHECK(backward_step.contact_load == bt::FootContactLoad::HeelOnly);
    BT_CHECK(backward_step.transition_quality > 0.50f);

    const auto noisy_heel_pose = Foot(0.22f, 1.27f, -0.1f);
    auto noisy_toe_to_heel = bt::UpdateFootSupport(
        toe_only,
        noisy_heel_pose,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        1.0 / 60.0,
        config,
        &heel_only_evidence);
    BT_CHECK(noisy_toe_to_heel.phase != bt::FootSupportPhase::HeelLock);
    BT_CHECK(noisy_toe_to_heel.transition_quality < 0.25f);

    bt::FootSupportEvidence split_full_evidence = heel_only_evidence;
    split_full_evidence.heel_confidence = 0.90f;
    split_full_evidence.toe_confidence = 0.92f;
    auto calibrated_split = bt::UpdateFootSupportCalibrated(
        {},
        Foot(0.0f, 1.25f, 0.0f),
        Foot(0.0f, 1.25f, 0.0f),
        bt::PostureMode::UprightStanding,
        floor,
        0.20,
        0.36f,
        config,
        &split_full_evidence);
    BT_CHECK(calibrated_split.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(calibrated_split.contact_load == bt::FootContactLoad::FullPlant);
    BT_CHECK_NEAR(calibrated_split.heel_anchor.pose.position.z, -0.18f, 1e-5);
    BT_CHECK_NEAR(calibrated_split.toe_anchor.pose.position.z, 0.18f, 1e-5);

    auto missing_hold = support;
    missing_hold.anchor.confidence = 0.65f;
    missing_hold = bt::UpdateFootSupport(
        missing_hold,
        foot,
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        config.release_seconds,
        config,
        &missing_evidence);
    BT_CHECK(missing_hold.type == bt::FootSupportType::FloorSupport);
    BT_CHECK(missing_hold.anchor.active);

    auto contradicted_release = support;
    contradicted_release.anchor.confidence = 0.65f;
    contradicted_release = bt::UpdateFootSupport(
        contradicted_release,
        Foot(0.2f, 1.55f, -0.1f),
        foot,
        bt::PostureMode::UprightStanding,
        floor,
        config.release_seconds * 0.5,
        config,
        &weak_evidence);
    BT_CHECK(contradicted_release.type == bt::FootSupportType::None);
    BT_CHECK(!contradicted_release.anchor.active);

    return 0;
}
