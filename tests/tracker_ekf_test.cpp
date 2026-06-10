#include "tracking/tracker_ekf.h"
#include "tracking/contact_constraints.h"
#include "test_check.h"

#include <cmath>

namespace {

bt::LowerBodyState State(float root_x, float left_x, float right_x, float confidence) {
    bt::LowerBodyState state;
    state.root.position = bt::Vec3f{root_x, 1.0f, 0.0f};
    state.left_foot.position = bt::Vec3f{left_x, 0.0f, -0.1f};
    state.right_foot.position = bt::Vec3f{right_x, 0.0f, 0.1f};
    state.root.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    state.left_foot.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    state.right_foot.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    state.confidence = confidence;
    return state;
}

bt::Quatf Yaw(float radians) {
    const float half = radians * 0.5f;
    return bt::Quatf{0.0f, std::sin(half), 0.0f, std::cos(half)};
}

float QuatLength(const bt::Quatf& q) {
    return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

float QuatDot(const bt::Quatf& a, const bt::Quatf& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

bool SameOrientation(const bt::Quatf& a, const bt::Quatf& b, float epsilon) {
    return std::abs(QuatDot(bt::Normalize(a), bt::Normalize(b))) > 1.0f - epsilon;
}

bt::FootSupportState LockedSupport(const bt::Pose3f& anchor, float confidence = 1.0f) {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = bt::FootSupportPhase::FlatPlant;
    support.anchor.active = true;
    support.anchor.pose = anchor;
    support.anchor.confidence = confidence;
    return support;
}

bt::FootSupportState ToePivotSupport(const bt::Pose3f& anchor) {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = bt::FootSupportPhase::ToePivot;
    support.anchor.active = true;
    support.anchor.pose = anchor;
    support.anchor.confidence = 1.0f;
    support.toe_anchor.active = true;
    support.toe_anchor.pose = anchor;
    support.toe_anchor.pose.position = bt::FootToeContactPoint(anchor);
    support.toe_anchor.confidence = 1.0f;
    return support;
}

} // namespace

int main() {
    bt::TrackerEkfConfig config;
    config.process_noise_mps2 = 2.0f;
    config.min_measurement_variance_m2 = 0.0001f;
    config.max_measurement_variance_m2 = 0.0400f;

    bt::TrackerEkfState filter;
    bt::TrackerEkfTelemetry telemetry;
    auto out = bt::ApplyTrackerEkf(filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK_NEAR(out.root.position.x, 0.0, 1e-5);
    BT_CHECK(filter.root.initialized);
    BT_CHECK(telemetry.enabled);
    BT_CHECK(telemetry.applied);
    BT_CHECK(!telemetry.reset);
    BT_CHECK(telemetry.root_initialized);
    BT_CHECK(telemetry.root.filtered);
    BT_CHECK_NEAR(telemetry.root.mean_position_gain, 1.0, 1e-6);

    out = bt::ApplyTrackerEkf(filter, State(1.0f, -0.2f, 0.2f, 0.10f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK(out.root.position.x < 1.0f);
    BT_CHECK(out.root.position.x > 0.0f);

    bt::TrackerEkfConfig gated_config = config;
    gated_config.mahalanobis_gate_enabled = true;
    gated_config.outlier_variance_scale = 128.0f;
    bt::TrackerEkfState gated_filter;
    bt::TrackerEkfTelemetry gated_telemetry;
    auto gated_out = bt::ApplyTrackerEkf(gated_filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, gated_config, &gated_telemetry);
    gated_out = bt::ApplyTrackerEkf(gated_filter, State(5.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, gated_config, &gated_telemetry);
    BT_CHECK(gated_telemetry.root.outlier_inflated);
    BT_CHECK(gated_telemetry.root.mahalanobis_chi2 > gated_config.mahalanobis_gate_chi2);

    bt::TrackerEkfConfig ungated_config = gated_config;
    ungated_config.mahalanobis_gate_enabled = false;
    bt::TrackerEkfState ungated_filter;
    bt::TrackerEkfTelemetry ungated_telemetry;
    auto ungated_out = bt::ApplyTrackerEkf(ungated_filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, ungated_config, &ungated_telemetry);
    ungated_out = bt::ApplyTrackerEkf(ungated_filter, State(5.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, ungated_config, &ungated_telemetry);
    BT_CHECK(gated_out.root.position.x < ungated_out.root.position.x);

    out = bt::ApplyTrackerEkf(filter, State(5.0f, -0.2f, 0.2f, 0.0f), 1.0 / 60.0, config, &telemetry);
    const float held_x = out.root.position.x;
    BT_CHECK(held_x < 5.0f);
    BT_CHECK(held_x > 0.0f);
    BT_CHECK(filter.root.initialized);
    BT_CHECK(telemetry.reset);
    BT_CHECK(telemetry.applied);

    bt::ResetTrackerEkf(filter);
    out = bt::ApplyTrackerEkf(filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    out = bt::ApplyTrackerEkf(filter, State(100.0f, -100.0f, 100.0f, 0.0f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK_NEAR(out.root.position.x, 0.0f, 1e-5f);
    BT_CHECK_NEAR(filter.root.axes[0].position, 0.0f, 1e-5f);

    config.missing_velocity_decay = 0.20f;
    bt::ResetTrackerEkf(filter);
    out = bt::ApplyTrackerEkf(filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    filter.root.axes[0].velocity = 1.0f;
    out = bt::ApplyTrackerEkf(filter, State(1.0f / 60.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK(filter.root.axes[0].velocity > 0.95f);
    out = bt::ApplyTrackerEkf(filter, State(10.0f, -0.2f, 0.2f, 0.0f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK(filter.root.axes[0].velocity < 0.25f);
    config.missing_velocity_decay = 0.92f;

    bt::ResetTrackerEkf(filter);
    out = bt::ApplyTrackerEkf(filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    out = bt::ApplyTrackerEkf(filter, State(1.0f, -0.2f, 0.2f, 0.10f), 1.0 / 60.0, config, &telemetry);
    out = bt::ApplyTrackerEkf(filter, State(5.0f, -0.2f, 0.2f, 0.0f), 1.0 / 60.0, config, &telemetry);

    out = bt::ApplyTrackerEkf(filter, State(3.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK(out.root.position.x < 3.0f);
    BT_CHECK(out.root.position.x > held_x);
    BT_CHECK(filter.root.initialized);

    bt::ResetTrackerEkf(filter);
    BT_CHECK(!filter.root.initialized);

    bt::TrackerEkfState disabled_filter;
    config.enabled = false;
    out = bt::ApplyTrackerEkf(disabled_filter, State(2.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    BT_CHECK_NEAR(out.root.position.x, 2.0, 1e-5);
    BT_CHECK(!disabled_filter.root.initialized);
    BT_CHECK(!telemetry.enabled);
    BT_CHECK(!telemetry.applied);

    config.enabled = true;
    bt::ResetTrackerEkf(filter);
    out = bt::ApplyTrackerEkf(filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    auto stale = State(0.0f, 0.90f, 0.2f, 1.0f);
    out = bt::ApplyTrackerEkf(filter, stale, 1.0 / 60.0, config, &telemetry);
    BT_CHECK(filter.left_foot.axes[0].position > 0.5f);

    auto locked = State(0.0f, 0.90f, 0.2f, 1.0f);
    bt::Pose3f anchor = locked.left_foot;
    anchor.position = bt::Vec3f{-0.25f, 0.0f, -0.1f};
    anchor.orientation = Yaw(0.50f);
    locked.support.left_foot = LockedSupport(anchor);
    out = bt::ApplyTrackerEkf(filter, locked, 1.0 / 60.0, config, &telemetry);
    BT_CHECK_NEAR(out.left_foot.position.x, anchor.position.x, 1e-6);
    BT_CHECK_NEAR(filter.left_foot.axes[0].position, anchor.position.x, 1e-6);
    BT_CHECK_NEAR(filter.left_foot.axes[0].velocity, 0.0, 1e-6);
    BT_CHECK(telemetry.left_foot.locked_reset);
    BT_CHECK(!telemetry.left_foot.filtered);

    bt::ResetTrackerEkf(filter);
    auto plant_entry_seed = State(0.0f, 0.05f, 0.2f, 1.0f);
    out = bt::ApplyTrackerEkf(filter, plant_entry_seed, 1.0 / 60.0, config, &telemetry);

    auto plant_entry = plant_entry_seed;
    plant_entry.left_foot.orientation = Yaw(0.0f);
    bt::Pose3f plant_anchor = plant_entry.left_foot;
    plant_anchor.position = bt::Vec3f{-0.25f, 0.0f, -0.1f};
    plant_anchor.orientation = Yaw(1.0f);
    plant_entry.support.left_foot = LockedSupport(plant_anchor, 0.25f);
    out = bt::ApplyTrackerEkf(filter, plant_entry, 1.0 / 60.0, config, &telemetry);
    BT_CHECK(telemetry.left_foot.locked_reset);
    BT_CHECK(out.left_foot.position.x > plant_anchor.position.x);
    BT_CHECK(out.left_foot.position.x < plant_entry.left_foot.position.x);
    BT_CHECK(!SameOrientation(out.left_foot.orientation, plant_anchor.orientation, 1e-5f));
    BT_CHECK(out.left_foot.orientation.y > 0.0f);
    BT_CHECK(out.left_foot.orientation.y < plant_anchor.orientation.y);
    BT_CHECK_NEAR(QuatLength(out.left_foot.orientation), 1.0, 1e-5);

    auto full_plant = plant_entry;
    full_plant.support.left_foot = LockedSupport(plant_anchor, 1.0f);
    out = bt::ApplyTrackerEkf(filter, full_plant, 1.0 / 60.0, config, &telemetry);
    BT_CHECK(!telemetry.left_foot.locked_reset);
    BT_CHECK_NEAR(out.left_foot.position.x, plant_anchor.position.x, 1e-6);
    BT_CHECK(SameOrientation(out.left_foot.orientation, plant_anchor.orientation, 1e-5f));

    auto released = State(0.0f, -0.26f, 0.2f, 1.0f);
    out = bt::ApplyTrackerEkf(filter, released, 1.0 / 60.0, config, &telemetry);
    BT_CHECK(out.left_foot.position.x < -0.24f);
    BT_CHECK(out.left_foot.position.x > -0.27f);


    bt::ResetTrackerEkf(filter);
    auto toe_pivot = State(0.0f, -0.20f, 0.2f, 1.0f);
    bt::Pose3f toe_anchor = toe_pivot.left_foot;
    toe_pivot.support.left_foot = ToePivotSupport(toe_anchor);
    toe_pivot.left_foot.position = bt::Vec3f{-0.20f, 0.071545f, -0.077500f};
    toe_pivot.left_foot.orientation = bt::Quatf{0.30f, 0.0f, 0.0f, 0.953939f};
    out = bt::ApplyTrackerEkf(filter, toe_pivot, 1.0 / 60.0, config, &telemetry);
    const bt::Vec3f toe_contact = bt::FootToeContactPoint(out.left_foot);
    const bt::Vec3f expected_toe_contact = toe_pivot.support.left_foot.toe_anchor.pose.position;
    BT_CHECK_NEAR(toe_contact.x, expected_toe_contact.x, 1e-5);
    BT_CHECK_NEAR(toe_contact.y, expected_toe_contact.y, 1e-5);
    BT_CHECK_NEAR(toe_contact.z, expected_toe_contact.z, 1e-5);
    BT_CHECK(telemetry.left_foot.filtered);

    bt::ResetTrackerEkf(filter);
    config.foot_orientation_gain = 0.35f;
    out = bt::ApplyTrackerEkf(filter, State(0.0f, -0.2f, 0.2f, 1.0f), 1.0 / 60.0, config, &telemetry);
    auto rotated = State(0.0f, -0.2f, 0.2f, 1.0f);
    rotated.left_foot.orientation = Yaw(1.57079632679f);
    out = bt::ApplyTrackerEkf(filter, rotated, 1.0 / 60.0, config, &telemetry);
    BT_CHECK(out.left_foot.orientation.y > 0.0f);
    BT_CHECK(out.left_foot.orientation.y < rotated.left_foot.orientation.y);
    BT_CHECK_NEAR(out.root.orientation.y, 0.0, 1e-6);
    BT_CHECK_NEAR(telemetry.left_foot.orientation_gain, config.foot_orientation_gain, 1e-6);

    return 0;
}
