#include "debug/replay_log.h"
#include "tracking/contact_constraints.h"
#include "test_check.h"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>

namespace {

bt::Pose3f Pose(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}

bt::FootSupportState FootSupport(const bt::Pose3f& foot) {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = bt::FootSupportPhase::ToePivot;
    support.anchor.active = true;
    support.anchor.pose = foot;
    support.anchor.confidence = 0.90f;
    support.heel_anchor.active = true;
    support.heel_anchor.pose = foot;
    support.heel_anchor.pose.position = bt::FootHeelContactPoint(foot);
    support.heel_anchor.confidence = 0.80f;
    support.toe_anchor.active = true;
    support.toe_anchor.pose = foot;
    support.toe_anchor.pose.position = bt::FootToeContactPoint(foot);
    support.toe_anchor.confidence = 0.85f;
    return support;
}

} // namespace

int main() {
    bt::DebugSnapshot snapshot;
    snapshot.phase = "runtime";
    snapshot.timestamp_seconds = 12.5;
    snapshot.tracking.state.root = Pose(0.10f, 0.90f, 0.0f);
    snapshot.tracking.state.left_foot = Pose(-0.20f, 0.0f, -0.10f);
    snapshot.tracking.state.right_foot = Pose(0.20f, 0.0f, 0.10f);
    snapshot.tracking.state.confidence = 0.95f;
    snapshot.tracking.state.support.root_support = bt::RootSupportType::FeetSupported;
    snapshot.tracking.state.support.root_anchor.active = true;
    snapshot.tracking.state.support.root_anchor.pose = snapshot.tracking.state.root;
    snapshot.tracking.state.support.root_anchor.confidence = 0.75f;
    snapshot.tracking.state.support.left_foot = FootSupport(snapshot.tracking.state.left_foot);
    snapshot.tracking.state.support.right_foot = FootSupport(snapshot.tracking.state.right_foot);
    snapshot.tracking.body_calibration.enabled = true;
    snapshot.tracking.body_calibration.auto_persist = true;
    snapshot.tracking.body_calibration.complete = true;
    snapshot.tracking.body_calibration.persisted = true;
    snapshot.tracking.body_calibration.persist_status = "saved_on_shutdown";
    snapshot.tracking.body_calibration.overall_confidence = 0.88f;
    snapshot.tracking.body_calibration.body.left_femur = 0.43f;
    snapshot.tracking.body_calibration.body.right_femur = 0.44f;
    snapshot.tracking.body_calibration.body.left_tibia = 0.41f;
    snapshot.tracking.body_calibration.body.right_tibia = 0.42f;
    snapshot.tracking.body_calibration.body.left_foot_length = 0.25f;
    snapshot.tracking.body_calibration.body.right_foot_length = 0.26f;

    snapshot.tracking.stages.predicted.valid = true;
    snapshot.tracking.stages.predicted.root = Pose(0.00f, 0.90f, 0.0f);
    snapshot.tracking.stages.measured.valid = true;
    snapshot.tracking.stages.measured.root = Pose(0.12f, 0.90f, 0.0f);
    snapshot.tracking.stages.motion_filtered.valid = true;
    snapshot.tracking.stages.motion_filtered.root = Pose(0.10f, 0.90f, 0.0f);
    snapshot.tracking.stages.ekf_filtered.valid = true;
    snapshot.tracking.stages.ekf_filtered.root = Pose(0.095f, 0.90f, 0.0f);
    snapshot.tracking.stages.corrected.valid = true;
    snapshot.tracking.stages.corrected.root = snapshot.tracking.state.root;

    auto& contact = snapshot.tracking.motion_filter.contact_root;
    contact.applied = true;
    contact.reason = bt::MotionFilterReason::ContactRootCommonMode;
    contact.correction = bt::Vec3f{-0.01f, 0.0f, 0.0f};
    contact.left_residual = bt::Vec3f{0.02f, 0.0f, 0.0f};
    contact.right_residual = bt::Vec3f{0.018f, 0.0f, 0.0f};
    contact.common_residual = bt::Vec3f{0.019f, 0.0f, 0.0f};
    contact.root_innovation = bt::Vec3f{0.021f, 0.0f, 0.0f};
    contact.correction_m = 0.01f;
    contact.left_residual_m = 0.02f;
    contact.right_residual_m = 0.018f;
    contact.common_residual_m = 0.019f;
    contact.root_innovation_m = 0.021f;
    contact.disagreement_m = 0.002f;
    contact.root_alignment = 0.99f;

    const auto path = std::filesystem::temp_directory_path() / "bodytracker_replay_export_test.ndjson";
    bt::ReplayLogWriter writer;
    bt::ReplayLogSessionInfo session;
    session.config_path = "config/default.json";
    session.config_json = R"json({"tracking":{"enable_replay_recording":true}})json";
    session.model_path = "models/rtmw-dw-x-l-cocktail14-384x288.onnx";
    session.calibration_path = "calib/default.json";
    auto status = writer.Open(path, session);
    BT_CHECK(status.ok());
    status = writer.WriteSnapshot(snapshot);
    BT_CHECK(status.ok());
    writer.Close();

    {
        std::ifstream in(path);
        BT_CHECK(static_cast<bool>(in));
        std::string line;
        std::getline(in, line);
        const auto manifest = nlohmann::json::parse(line);
        BT_CHECK(manifest.at("type").get<std::string>() == "replay_manifest");
        BT_CHECK(manifest.at("config").at("json").get<std::string>().find("enable_replay_recording") != std::string::npos);
        std::getline(in, line);
        const auto j = nlohmann::json::parse(line);
        BT_CHECK(j.at("type").get<std::string>() == "replay_frame");
        BT_CHECK(j.at("schema_version").get<int>() == session.schema_version);
        BT_CHECK(j.at("frame_index").get<int>() == 0);

        const auto& tracking = j.at("tracking");
        const auto& exported_contact = tracking.at("motion_filter").at("contact_root");
        BT_CHECK(exported_contact.at("applied").get<bool>());
        BT_CHECK(exported_contact.at("reason").get<std::string>() == "CONTACT_ROOT_COMMON_MODE");
        BT_CHECK_NEAR(exported_contact.at("correction").at(0).get<float>(), -0.01f, 1e-6);
        BT_CHECK_NEAR(exported_contact.at("left_residual").at(0).get<float>(), 0.02f, 1e-6);
        BT_CHECK_NEAR(exported_contact.at("right_residual").at(0).get<float>(), 0.018f, 1e-6);
        BT_CHECK_NEAR(exported_contact.at("common_residual_m").get<float>(), 0.019f, 1e-6);
        BT_CHECK_NEAR(exported_contact.at("root_innovation_m").get<float>(), 0.021f, 1e-6);

        BT_CHECK(tracking.at("support").at("left_foot").at("toe_anchor").at("active").get<bool>());
        BT_CHECK(tracking.at("support").at("left_foot").contains("heel_contact_point"));
        BT_CHECK(tracking.at("support").at("left_foot").contains("toe_contact_point"));
        BT_CHECK(tracking.at("stages").at("predicted").at("valid").get<bool>());
        BT_CHECK(tracking.at("stages").at("measured").at("valid").get<bool>());
        BT_CHECK(tracking.at("stages").at("motion_filtered").at("valid").get<bool>());
        BT_CHECK(tracking.at("stages").at("ekf_filtered").at("valid").get<bool>());
        BT_CHECK(tracking.at("stages").at("corrected").at("valid").get<bool>());
        const auto& body_calibration = tracking.at("body_calibration");
        BT_CHECK(body_calibration.at("auto_persist").get<bool>());
        BT_CHECK(body_calibration.at("complete").get<bool>());
        BT_CHECK(body_calibration.at("persisted").get<bool>());
        BT_CHECK(!body_calibration.at("persist_pending").get<bool>());
        BT_CHECK(body_calibration.at("persist_status").get<std::string>() == "saved_on_shutdown");
        BT_CHECK_NEAR(body_calibration.at("body").at("left_femur").get<float>(), 0.43f, 1e-6);
        BT_CHECK_NEAR(body_calibration.at("body").at("right_foot_length").get<float>(), 0.26f, 1e-6);
    }

    std::error_code cleanup_error;
    const bool removed = std::filesystem::remove(path, cleanup_error);
    BT_CHECK(!cleanup_error);
    BT_CHECK(removed);
    return 0;
}
