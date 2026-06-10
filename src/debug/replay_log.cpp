#include "debug/replay_log.h"

#include "tracking/contact_constraints.h"
#include "tracking/steamvr_alignment_json.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
#else
#include "nlohmann_json.hpp"
#endif
#include <iomanip>
#include <sstream>
#include <string>

namespace bt {
namespace {

template <typename T, std::size_t N>
nlohmann::json ArrayToJson(const std::array<T, N>& values) {
    auto out = nlohmann::json::array();
    for (const auto& value : values) out.push_back(value);
    return out;
}

nlohmann::json Vec3ToJson(const Vec3f& v) {
    return {v.x, v.y, v.z};
}

nlohmann::json QuatToJson(const Quatf& q) {
    return {q.x, q.y, q.z, q.w};
}

nlohmann::json PoseToJson(const Pose3f& pose) {
    return {
        {"position", Vec3ToJson(pose.position)},
        {"orientation", QuatToJson(pose.orientation)}
    };
}

nlohmann::json Vec2ToJson(const Vec2f& v) {
    return {v.x, v.y};
}

nlohmann::json StereoHmdAnchorToJson(const StereoHmdAnchorResult& anchor) {
    return {
        {"state", ToString(anchor.state)},
        {"reason", anchor.reason},
        {"applied", anchor.applied},
        {"due", anchor.due},
        {"interval_seconds", anchor.interval_seconds},
        {"seconds_since_last_anchor", anchor.seconds_since_last_anchor},
        {"hmd_world", Vec3ToJson(anchor.hmd_world)},
        {"stereo_head_world", Vec3ToJson(anchor.stereo_head_world)},
        {"correction_world", Vec3ToJson(anchor.correction_world)},
        {"correction_m", anchor.correction_m},
        {"corrected_root_world", Vec3ToJson(anchor.corrected_root_world)}
    };
}

nlohmann::json ProjectionCorrectionToJson(const ProjectionCorrection& correction) {
    return {{"valid", correction.valid}, {"used_live_anchor", correction.used_live_anchor}, {"used_room_prior", correction.used_room_prior}, {"usable_for_room_map_update", correction.usable_for_room_map_update}, {"anchors_used", correction.anchors_used}, {"mode", correction.mode}, {"fallback_reason", correction.fallback_reason}, {"depth_scale", correction.depth_scale}, {"translation_delta_world", Vec3ToJson(correction.translation_delta_world)}, {"rotation_delta_world", QuatToJson(correction.rotation_delta_world)}, {"reprojection_error_px", correction.reprojection_error_px}, {"max_reprojection_error_px", correction.max_reprojection_error_px}, {"camera_anchor_timestamp_delta_ms", correction.camera_anchor_timestamp_delta_ms}};
}

nlohmann::json RoomDepthMapToJson(const RoomDepthMapTelemetry& map) {
    return {{"state", map.state}, {"accepted_frames", map.accepted_frames}, {"rejected_frames", map.rejected_frames}, {"coverage", map.coverage}, {"mean_variance_m2", map.mean_variance_m2}, {"last_accepted_update_time_seconds", map.last_accepted_update_time_seconds}, {"last_rejection_reason", map.last_rejection_reason}};
}

nlohmann::json HmdDepthScaleToJson(const HmdDepthScaleResult& scale) {
    return {
        {"state", ToString(scale.state)},
        {"reason", scale.reason},
        {"live", scale.live},
        {"held", scale.held},
        {"usable", scale.usable},
        {"scale", scale.scale},
        {"corrected_head_world", Vec3ToJson(scale.corrected_head_world)},
        {"corrected_root_world", Vec3ToJson(scale.corrected_root_world)},
        {"corrected_root_valid", scale.corrected_root_valid},
        {"observation", {
            {"valid", scale.observation.valid},
            {"head_keypoint", ToString(scale.observation.head_keypoint)},
            {"head_px", Vec2ToJson(scale.observation.head_px)},
            {"head_ray_camera_unit", Vec3ToJson(scale.observation.head_ray_camera_unit)},
            {"hmd_world", Vec3ToJson(scale.observation.hmd_world)},
            {"hmd_camera", Vec3ToJson(scale.observation.hmd_camera)},
            {"mono_head_depth_m", scale.observation.mono_head_depth_m},
            {"true_head_depth_z_m", scale.observation.true_head_depth_z_m},
            {"scale", scale.observation.scale},
            {"camera_hmd_timestamp_delta_ms", scale.observation.camera_hmd_timestamp_delta_ms}
        }}
    };
}

bool OscTrackerSpaceStale(const std::string& source) {
    return source == "steamvr_controller_alignment_stale";
}

std::string OscTrackerSpaceState(const DebugSnapshot& snapshot) {
    if (OscTrackerSpaceStale(snapshot.osc_tracker_space_source)) {
        return "stale_controller_alignment";
    }
    if (!snapshot.osc_tracker_space_transform_valid) {
        return "missing_or_invalid_transform";
    }
    return "valid_active_transform";
}


std::string StableTextHash(const std::string& text) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const unsigned char ch : text) {
        hash ^= static_cast<std::uint64_t>(ch);
        hash *= 1099511628211ull;
    }
    std::ostringstream out;
    out << std::hex << std::setw(16) << std::setfill('0') << hash;
    return out.str();
}

nlohmann::json ReplayManifestToJson(const ReplayLogSessionInfo& session) {
    return {
        {"type", "replay_manifest"},
        {"format", "bodytracker.ndjson.replay"},
        {"schema_version", session.schema_version},
        {"deterministic", session.deterministic},
        {"config", {
            {"path", session.config_path},
            {"json", session.config_json},
            {"hash_fnv1a64", StableTextHash(session.config_json)},
            {"recorded", !session.config_json.empty()}
        }},
        {"model_path", session.model_path},
        {"calibration_path", session.calibration_path}
    };
}

nlohmann::json RoiToJson(const RoiState& roi) {
    return {
        {"initialized", roi.initialized},
        {"locked", roi.pose_locked},
        {"reacquire", roi.in_reacquire},
        {"confidence", roi.confidence},
        {"rect", {roi.rect.x, roi.rect.y, roi.rect.width, roi.rect.height}},
        {"stable_updates", roi.stable_updates},
        {"lost_updates", roi.lost_updates},
        {"contributing_points", roi.diagnostics.contributing_points}
    };
}

const char* TrackerRoleName(TrackerRole role) {
    return ToString(role);
}

const char* MotionTargetName(MotionTarget target) {
    switch (target) {
    case MotionTarget::Root: return "root";
    case MotionTarget::LeftFoot: return "left_foot";
    case MotionTarget::RightFoot: return "right_foot";
    default: return "unknown";
    }
}

bool AnyValidTracker(const TrackerPoseArray& trackers) {
    for (const auto& tracker : trackers) {
        if (tracker.valid) {
            return true;
        }
    }
    return false;
}

bool AcceptedRuntimeProjectiveFloorGeometry(const FloorGeometryCalibration& geometry) {
    return geometry.valid && geometry.homography_valid && geometry.metric_scale_confidence > 0.0f;
}

bool AcceptedRuntimeScalarFloorGeometry(const FloorGeometryCalibration& geometry) {
    return geometry.valid &&
        geometry.family_a.valid &&
        geometry.family_a.metric_spacing_valid &&
        geometry.family_a.spacing_m > 0.0f &&
        geometry.family_a.spacing_px > 1.0f;
}

bool AcceptedRuntimeFloorGeometry(const FloorGeometryCalibration& geometry) {
    return AcceptedRuntimeProjectiveFloorGeometry(geometry) ||
        AcceptedRuntimeScalarFloorGeometry(geometry);
}

std::string AcceptedFloorGeometrySource(const FloorGeometryCalibration& geometry) {
    if (AcceptedRuntimeProjectiveFloorGeometry(geometry)) {
        return "floor_projective";
    }
    if (AcceptedRuntimeScalarFloorGeometry(geometry)) {
        return "floor_spacing";
    }
    return "nothing";
}

const char* MonocularFloorAssistStatus(
    const BodySolveStereoTelemetry& stereo,
    const FloorGeometryCalibration& floor_geometry) {
    const bool non_monocular = stereo.tracking_mode != TrackingMode::Monocular;
    const bool accepted_runtime_geometry = AcceptedRuntimeFloorGeometry(floor_geometry);
    if (non_monocular && accepted_runtime_geometry) {
        return "standby";
    }
    if (non_monocular) {
        return "disabled";
    }
    const bool used_floor_depth =
        stereo.monocular_scale_source == MonocularScaleSource::FloorSpacing ||
        stereo.monocular_scale_source == MonocularScaleSource::FloorProjective ||
        stereo.monocular_scale_source == MonocularScaleSource::WallDepth ||
        stereo.floor_geometry_used;
    if (used_floor_depth || accepted_runtime_geometry) {
        return "active";
    }
    return "inactive";
}

TrackingSolverTelemetry EffectiveSolverTelemetry(const DebugSnapshot& snapshot) {
    if (snapshot.solver.used_hmd || snapshot.solver.degraded || !snapshot.solver.reason.empty()) {
        return snapshot.solver;
    }
    return snapshot.tracking.solver;
}

const TrackerPoseArray& EffectiveTrackers(const DebugSnapshot& snapshot) {
    return AnyValidTracker(snapshot.trackers) ? snapshot.trackers : snapshot.tracking.trackers;
}

const TrackerPose* FindTracker(const TrackerPoseArray& trackers, TrackerRole role) {
    const std::size_t role_index = TrackerRoleIndex(role);
    if (role_index >= trackers.size()) {
        return nullptr;
    }
    const auto& tracker = trackers[role_index];
    return tracker.valid ? &tracker : nullptr;
}

Pose3f EffectivePose(const DebugSnapshot& snapshot, TrackerRole role, const Pose3f& fallback) {
    if (const auto* tracker = FindTracker(EffectiveTrackers(snapshot), role)) {
        return tracker->pose;
    }
    return fallback;
}

float Clamp01ForTelemetry(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::clamp(value, 0.0f, 1.0f);
}

float EffectiveTrackingConfidence(const DebugSnapshot& snapshot) {
    if (std::isfinite(snapshot.tracking.state.confidence) && snapshot.tracking.state.confidence > 0.0f) {
        return Clamp01ForTelemetry(snapshot.tracking.state.confidence);
    }
    const auto& trackers = EffectiveTrackers(snapshot);
    float sum = 0.0f;
    int count = 0;
    for (const auto& tracker : trackers) {
        if (tracker.valid && std::isfinite(tracker.confidence)) {
            sum += Clamp01ForTelemetry(tracker.confidence);
            ++count;
        }
    }
    return count > 0 ? Clamp01ForTelemetry(sum / static_cast<float>(count)) : 0.0f;
}

nlohmann::json Keypoint2dToJson(const Keypoint2D& kp, std::size_t index) {
    const auto id = static_cast<KeypointId>(index);
    return {
        {"id", index},
        {"name", ToString(id)},
        {"present", kp.present},
        {"confidence", kp.confidence},
        {"pixel", {kp.pixel.x, kp.pixel.y}}
    };
}

nlohmann::json Pose2dToJson(const DecodedPose2D& pose) {
    nlohmann::json keypoints = nlohmann::json::array();
    for (std::size_t i = 0; i < pose.keypoints.size(); ++i) {
        keypoints.push_back(Keypoint2dToJson(pose.keypoints[i], i));
    }
    return {
        {"valid", pose.valid},
        {"format", ToString(pose.format)},
        {"aggregate_confidence", pose.aggregate_confidence},
        {"keypoints", keypoints}
    };
}

nlohmann::json ReliabilityToJson(const ReliabilitySummary& reliability) {
    nlohmann::json joints = nlohmann::json::array();
    for (std::size_t i = 0; i < reliability.joints.size(); ++i) {
        const auto id = static_cast<KeypointId>(i);
        const auto& joint = reliability.joints[i];
        joints.push_back({
            {"id", i},
            {"name", ToString(id)},
            {"usable", joint.usable},
            {"final_weight", joint.final_weight},
            {"model_term", joint.model_term},
            {"crop_edge_term", joint.crop_edge_term},
            {"image_edge_term", joint.image_edge_term},
            {"temporal_term", joint.temporal_term},
            {"crop_stability_term", joint.crop_stability_term},
            {"posture_mode_term", joint.posture_mode_term}
        });
    }
    return {
        {"mean_weight", reliability.mean_weight},
        {"lower_body_mean", reliability.lower_body_mean},
        {"foot_mean", reliability.foot_mean},
        {"joints", joints}
    };
}

nlohmann::json TrackerEvidenceToJson(const TrackerEvidence& evidence) {
    return {
        {"source", ToString(evidence.source)},
        {"direct_confidence", evidence.direct_confidence},
        {"support_confidence", evidence.support_confidence},
        {"anchor_held", evidence.anchor_held},
        {"valid", evidence.valid}
    };
}

nlohmann::json TrackersToJson(const TrackerPoseArray& trackers) {
    auto out = nlohmann::json::array();
    for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
        const auto& tracker = trackers[i];
        out.push_back({
            {"role", TrackerRoleName(kTrackerRoles[i])},
            {"valid", tracker.valid},
            {"confidence", tracker.confidence},
            {"evidence", TrackerEvidenceToJson(tracker.evidence)},
            {"pose", PoseToJson(tracker.pose)}
        });
    }
    return out;
}

nlohmann::json OscToJson(const DebugSnapshot& snapshot) {
    auto roles = nlohmann::json::array();
    auto active_roles = nlohmann::json::array();
    for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
        const int tracker_index = snapshot.osc_role_indices[i];
        const std::string base_address = tracker_index > 0
            ? "/tracking/trackers/" + std::to_string(tracker_index)
            : std::string{};
        roles.push_back({
            {"role", TrackerRoleName(kTrackerRoles[i])},
            {"tracker_index", tracker_index},
            {"configured", snapshot.osc_role_configured[i]},
            {"osc_address_position", base_address.empty() ? std::string{} : base_address + "/position"},
            {"osc_address_rotation", base_address.empty() ? std::string{} : base_address + "/rotation"},
            {"mapping_kind", "configured_vrchat_index_path"},
            {"vrchat_role_binding_validated", false},
            {"valid", snapshot.osc_role_valid[i]},
            {"sent", snapshot.osc_role_sent[i]},
            {"degraded", snapshot.osc_role_degraded[i]},
            {"reason", snapshot.osc_role_reasons[i]},
            {"error_detail", snapshot.osc_role_error_details[i]}
        });
        if (snapshot.osc_role_sent[i]) {
            active_roles.push_back(TrackerRoleName(kTrackerRoles[i]));
        }
    }
    return {
        {"enabled", snapshot.osc_enabled},
        {"open", snapshot.osc_open},
        {"status", snapshot.osc_status},
        {"target_address", snapshot.osc_target_address},
        {"target_port", snapshot.osc_target_port},
        {"tracker_space_transform_valid", snapshot.osc_tracker_space_transform_valid},
        {"tracker_space_source", snapshot.osc_tracker_space_source},
        {"tracker_space_state", OscTrackerSpaceState(snapshot)},
        {"tracker_space_stale", OscTrackerSpaceStale(snapshot.osc_tracker_space_source)},
        {"tracker_space_blocked", snapshot.osc_status == "blocked_tracker_space"},
        {"tracker_space_block_reason", snapshot.osc_status == "blocked_tracker_space" ? snapshot.osc_last_error : ""},
        {"manual_tracker_space_fallback_available", snapshot.osc_manual_tracker_space_fallback_valid},
        {"manual_tracker_space_fallback_valid", snapshot.osc_manual_tracker_space_fallback_valid},
        {"manual_tracker_space_source", snapshot.osc_manual_tracker_space_source},
        {"last_send_ok", snapshot.osc_last_send_ok},
        {"last_error", snapshot.osc_last_error},
        {"open_attempts", snapshot.osc_open_attempts},
        {"sent_tracker_count", snapshot.osc_sent_tracker_count},
        {"skipped_tracker_count", snapshot.osc_skipped_tracker_count},
        {"sent_message_count", snapshot.osc_sent_message_count},
        {"active_roles", active_roles},
        {"roles", roles}
    };
}

nlohmann::json SteamVrAlignmentToJson(const DebugSnapshot& snapshot) {
    if (!snapshot.steamvr_alignment_recorded) {
        return {
            {"recorded", false},
            {"reason", "steamvr alignment diagnostics were not recorded in this frame"}
        };
    }
    auto out = AlignmentStatusToJson(snapshot.steamvr_alignment);
    out["recorded"] = true;
    return out;
}


nlohmann::json BodyCalibrationQualityToJson(const BodyCalibrationQuality& q) {
    return {
        {"pelvis_width", q.pelvis_width},
        {"left_femur", q.left_femur},
        {"right_femur", q.right_femur},
        {"left_tibia", q.left_tibia},
        {"right_tibia", q.right_tibia},
        {"left_foot_length", q.left_foot_length},
        {"right_foot_length", q.right_foot_length},
        {"standing_hmd_to_pelvis", q.standing_hmd_to_pelvis},
        {"overall", q.overall},
        {"sample_count", q.sample_count},
        {"source", q.source}
    };
}

nlohmann::json BodyCalibrationToJson(const BodyCalibration& body) {
    return {
        {"standing_neutral_valid", body.standing_neutral_valid},
        {"pelvis_width", body.pelvis_width},
        {"left_femur", body.left_femur},
        {"right_femur", body.right_femur},
        {"left_tibia", body.left_tibia},
        {"right_tibia", body.right_tibia},
        {"left_foot_length", body.left_foot_length},
        {"right_foot_length", body.right_foot_length},
        {"standing_hmd_to_pelvis", Vec3ToJson(body.standing_hmd_to_pelvis)},
        {"quality", BodyCalibrationQualityToJson(body.quality)}
    };
}

nlohmann::json BodyCalibrationTelemetryToJson(const BodyCalibrationTelemetry& telemetry) {
    return {
        {"enabled", telemetry.enabled},
        {"auto_persist", telemetry.auto_persist},
        {"complete", telemetry.complete},
        {"saved_this_frame", telemetry.saved_this_frame},
        {"persisted", telemetry.persisted},
        {"persist_pending", telemetry.persist_pending},
        {"used_stereo", telemetry.used_stereo},
        {"used_monocular_floor_scale", telemetry.used_monocular_floor_scale},
        {"accumulated_seconds", telemetry.accumulated_seconds},
        {"accepted_samples", telemetry.accepted_samples},
        {"overall_confidence", telemetry.overall_confidence},
        {"reason", telemetry.reason},
        {"persist_status", telemetry.persist_status},
        {"persist_error", telemetry.persist_error},
        {"body", BodyCalibrationToJson(telemetry.body)}
    };
}


nlohmann::json FloorLineFamilyToJson(const FloorGeometryLineFamily& f) {
    return {
        {"valid", f.valid},
        {"confidence", f.confidence},
        {"orientation_rad", f.orientation_rad},
        {"spacing_px", f.spacing_px},
        {"spacing_m", f.spacing_m},
        {"metric_spacing_valid", f.metric_spacing_valid},
        {"reference_rho_px", f.reference_rho_px},
        {"vanishing_point_px", {f.vanishing_point_px.x, f.vanishing_point_px.y}},
        {"vanishing_point_valid", f.vanishing_point_valid},
        {"accepted_line_count", f.accepted_line_count},
        {"rejected_line_count", f.rejected_line_count},
        {"reason", f.reason}
    };
}

std::string FloorGeometrySource(const FloorGeometryCalibration& g) {
    if (!g.valid) {
        return "nothing";
    }
    return g.source.empty() || g.source == "unknown" ? "legacy_json" : g.source;
}

nlohmann::json FloorGeometryToJson(const FloorGeometryCalibration& g) {
    return {
        {"valid", g.valid},
        {"source", FloorGeometrySource(g)},
        {"image_width", g.image_width},
        {"image_height", g.image_height},
        {"floor_type", g.floor_type},
        {"family_count", g.family_count},
        {"family_a", FloorLineFamilyToJson(g.family_a)},
        {"family_b", FloorLineFamilyToJson(g.family_b)},
        {"two_axis_grid_valid", g.two_axis_grid_valid},
        {"homography_valid", g.homography_valid},
        {"homography_reprojection_error_px", g.homography_reprojection_error_px},
        {"homography_inlier_count", g.homography_inlier_count},
        {"homography_intersection_count", g.homography_intersection_count},
        {"homography_reason", g.homography_reason},
        {"floor_from_image", ArrayToJson(g.floor_from_image)},
        {"image_from_floor", ArrayToJson(g.image_from_floor)},
        {"floor_plane", {
            {"valid", g.floor_plane.valid},
            {"normal", Vec3ToJson(g.floor_plane.normal)},
            {"distance", g.floor_plane.distance}
        }},
        {"floor_plane_confidence", g.floor_plane_confidence},
        {"camera_orientation_valid", g.camera_orientation_valid},
        {"camera_pitch_rad", g.camera_pitch_rad},
        {"camera_roll_rad", g.camera_roll_rad},
        {"camera_yaw_rad", g.camera_yaw_rad},
        {"camera_orientation_confidence", g.camera_orientation_confidence},
        {"camera_orientation_applied_to_runtime", g.camera_orientation_applied_to_runtime},
        {"camera_height_valid", g.camera_height_valid},
        {"camera_height_m", g.camera_height_m},
        {"metric_scale_confidence", g.metric_scale_confidence},
        {"distortion", {
            {"available", g.distortion.available},
            {"valid", g.distortion.valid},
            {"applied_to_runtime", g.distortion.applied_to_runtime},
            {"confidence", g.distortion.confidence},
            {"radial_k1", g.distortion.radial_k1},
            {"radial_k2", g.distortion.radial_k2},
            {"tangential_p1", g.distortion.tangential_p1},
            {"tangential_p2", g.distortion.tangential_p2},
            {"straightness_error_px", g.distortion.straightness_error_px},
            {"corrected_straightness_error_px", g.distortion.corrected_straightness_error_px},
            {"sampled_seam_count", g.distortion.sampled_seam_count},
            {"sampled_point_count", g.distortion.sampled_point_count},
            {"model", g.distortion.model},
            {"reason", g.distortion.reason}
        }},
        {"multi_camera_alignment_confidence", g.multi_camera_alignment_confidence},
        {"multi_camera_alignment_valid", g.multi_camera_alignment_valid},
        {"multi_camera_warning", g.multi_camera_warning},
        {"multi_camera_yaw_delta_rad", g.multi_camera_yaw_delta_rad},
        {"multi_camera_pitch_delta_rad", g.multi_camera_pitch_delta_rad},
        {"multi_camera_roll_delta_rad", g.multi_camera_roll_delta_rad},
        {"multi_camera_height_delta_m", g.multi_camera_height_delta_m},
        {"multi_camera_scale_ratio", g.multi_camera_scale_ratio},
        {"shared_floor_frame_valid", g.shared_floor_frame_valid},
        {"shared_floor_transform", ArrayToJson(g.shared_floor_transform)},
        {"planted_drift_axis_confidence", g.planted_drift_axis_confidence},
        {"reason", g.reason}
    };
}

nlohmann::json AnchorToJson(const SupportAnchor& anchor) {
    return {
        {"active", anchor.active},
        {"confidence", anchor.confidence},
        {"dwell_seconds", anchor.dwell_seconds},
        {"release_seconds", anchor.release_seconds},
        {"pose", PoseToJson(anchor.pose)}
    };
}

float EffectiveFootSupportConfidence(const FootSupportState& support) {
    float confidence = support.anchor.active ? Clamp01ForTelemetry(support.anchor.confidence) : 0.0f;
    if (support.heel_anchor.active) {
        confidence = std::max(confidence, Clamp01ForTelemetry(support.heel_anchor.confidence));
    }
    if (support.toe_anchor.active) {
        confidence = std::max(confidence, Clamp01ForTelemetry(support.toe_anchor.confidence));
    }
    return Clamp01ForTelemetry(confidence);
}

nlohmann::json ContactResidualToJson(const FootContactResidual& residual) {
    return {
        {"valid", residual.valid},
        {"vector", Vec3ToJson(residual.residual)},
        {"magnitude_m", residual.magnitude_m}
    };
}

nlohmann::json SolveConstraintResidualToJson(const BodySolveConstraintResidualTelemetry& residual) {
    return {
        {"active", residual.active},
        {"weight", residual.weight},
        {"residual", residual.residual_m},
        {"score", residual.score}
    };
}

nlohmann::json FootSolveConstraintsToJson(const BodySolveFootConstraintTelemetry& foot) {
    return {
        {"support_confidence", foot.support_confidence},
        {"transition_quality", foot.transition_quality},
        {"floor_weight_scale", foot.floor_weight_scale},
        {"body_weight_scale", foot.body_weight_scale},
        {"heel_anchor", SolveConstraintResidualToJson(foot.heel_anchor)},
        {"toe_anchor", SolveConstraintResidualToJson(foot.toe_anchor)},
        {"full_plant", SolveConstraintResidualToJson(foot.full_plant)},
        {"floor_penetration", SolveConstraintResidualToJson(foot.floor_penetration)},
        {"sliding_velocity", SolveConstraintResidualToJson(foot.sliding_velocity)},
        {"orientation", SolveConstraintResidualToJson(foot.orientation)},
        {"degraded_or_released", foot.degraded_or_released}
    };
}

nlohmann::json SupportSolveConstraintsToJson(const BodySolveSupportConstraintTelemetry& constraints) {
    return {
        {"floor_calibration_weight", constraints.floor_calibration_weight},
        {"leg_length_weight", constraints.leg_length_weight},
        {"left_foot_length_weight", constraints.left_foot_length_weight},
        {"right_foot_length_weight", constraints.right_foot_length_weight},
        {"body_calibration_present", constraints.body_calibration_present},
        {"body_calibration_confidence", constraints.body_calibration_confidence},
        {"body_calibration_sample_count", constraints.body_calibration_sample_count},
        {"left_reach_clamped", constraints.left_reach_clamped},
        {"right_reach_clamped", constraints.right_reach_clamped},
        {"bone_length", SolveConstraintResidualToJson(constraints.bone_length)},
        {"root_support", SolveConstraintResidualToJson(constraints.root_support)},
        {"left_knee_floor_anchor", SolveConstraintResidualToJson(constraints.left_knee_floor_anchor)},
        {"right_knee_floor_anchor", SolveConstraintResidualToJson(constraints.right_knee_floor_anchor)},
        {"left_foot", FootSolveConstraintsToJson(constraints.left_foot)},
        {"right_foot", FootSolveConstraintsToJson(constraints.right_foot)}
    };
}

nlohmann::json AnchorResidualToJson(const SupportAnchor& anchor, const Vec3f& measured_contact) {
    if (!anchor.active) {
        return {
            {"valid", false},
            {"vector", Vec3ToJson(Vec3f{})},
            {"magnitude_m", 0.0f}
        };
    }

    const Vec3f residual = Sub(measured_contact, anchor.pose.position);
    return {
        {"valid", true},
        {"vector", Vec3ToJson(residual)},
        {"magnitude_m", Length(residual)}
    };
}

nlohmann::json FootSupportToJson(const FootSupportState& support, const Pose3f& foot_pose, float foot_length_m) {
    const Vec3f heel_contact = FootHeelContactPoint(foot_pose, foot_length_m);
    const Vec3f toe_contact = FootToeContactPoint(foot_pose, foot_length_m);
    return {
        {"type", ToString(support.type)},
        {"phase", ToString(support.phase)},
        {"contact_load", ToString(support.contact_load)},
        {"support_confidence", EffectiveFootSupportConfidence(support)},
        {"transition_quality", support.transition_quality},
        {"heel_contact_confidence", support.heel_contact_confidence},
        {"toe_contact_confidence", support.toe_contact_confidence},
        {"anchor", AnchorToJson(support.anchor)},
        {"heel_anchor", AnchorToJson(support.heel_anchor)},
        {"toe_anchor", AnchorToJson(support.toe_anchor)},
        {"heel_contact_point", Vec3ToJson(heel_contact)},
        {"toe_contact_point", Vec3ToJson(toe_contact)},
        {"contact_residual", ContactResidualToJson(FootSupportResidual(foot_pose, support, foot_length_m))},
        {"anchor_residual", AnchorResidualToJson(support.anchor, foot_pose.position)},
        {"heel_residual", AnchorResidualToJson(support.heel_anchor, heel_contact)},
        {"toe_residual", AnchorResidualToJson(support.toe_anchor, toe_contact)}
    };
}

nlohmann::json LegDofsToJson(const LowerBodyState& state) {
    return {
        {"left", {
            {"hip_flexion", state.left_hip_flexion},
            {"hip_abduction", state.left_hip_abduction},
            {"knee_flexion", state.left_knee_flexion},
            {"ankle_pitch", state.left_ankle_pitch},
            {"ankle_roll", state.left_ankle_roll},
            {"ankle_yaw", state.left_ankle_yaw},
            {"evidence", TrackerEvidenceToJson(state.left_foot_evidence)}
        }},
        {"right", {
            {"hip_flexion", state.right_hip_flexion},
            {"hip_abduction", state.right_hip_abduction},
            {"knee_flexion", state.right_knee_flexion},
            {"ankle_pitch", state.right_ankle_pitch},
            {"ankle_roll", state.right_ankle_roll},
            {"ankle_yaw", state.right_ankle_yaw},
            {"evidence", TrackerEvidenceToJson(state.right_foot_evidence)}
        }}
    };
}

nlohmann::json StagePoseToJson(const TrackingStagePoseSnapshot& stage) {
    return {
        {"valid", stage.valid},
        {"confidence", stage.confidence},
        {"root", PoseToJson(stage.root)},
        {"left_foot", PoseToJson(stage.left_foot)},
        {"right_foot", PoseToJson(stage.right_foot)}
    };
}

nlohmann::json TrackingStagesToJson(const TrackingPipelineStages& stages) {
    return {
        {"predicted", StagePoseToJson(stages.predicted)},
        {"preliminary", StagePoseToJson(stages.preliminary)},
        {"support_ready", StagePoseToJson(stages.support_ready)},
        {"measured", StagePoseToJson(stages.measured)},
        {"motion_filtered", StagePoseToJson(stages.motion_filtered)},
        {"ekf_filtered", StagePoseToJson(stages.ekf_filtered)},
        {"corrected", StagePoseToJson(stages.corrected)}
    };
}

nlohmann::json MotionFilterToJson(const MotionConsistencyTelemetry& telemetry) {
    nlohmann::json out = nlohmann::json::object();
    constexpr std::array<MotionTarget, 3> targets{
        MotionTarget::Root,
        MotionTarget::LeftFoot,
        MotionTarget::RightFoot
    };
    for (const auto target : targets) {
        const auto& entry = telemetry.targets[static_cast<std::size_t>(target)];
        out[MotionTargetName(target)] = {
            {"decision", ToString(entry.decision)},
            {"reason", ToString(entry.reason)},
            {"measured_distance_m", entry.measured_distance_m},
            {"expected_distance_m", entry.expected_distance_m},
            {"direction_deviation_deg", entry.direction_deviation_deg},
            {"lateral_deviation_ratio", entry.lateral_deviation_ratio},
            {"speed_change_ratio", entry.speed_change_ratio},
            {"direction_limit_deg", entry.direction_limit_deg},
            {"lateral_limit_ratio", entry.lateral_limit_ratio},
            {"speed_change_limit_ratio", entry.speed_change_limit_ratio},
            {"pending_frames", entry.pending_frames},
            {"confirm_frames", entry.confirm_frames}
        };
    }

    const auto& contact = telemetry.contact_root;
    out["contact_root"] = {
        {"applied", contact.applied},
        {"reason", ToString(contact.reason)},
        {"correction", Vec3ToJson(contact.correction)},
        {"left_residual", Vec3ToJson(contact.left_residual)},
        {"right_residual", Vec3ToJson(contact.right_residual)},
        {"common_residual", Vec3ToJson(contact.common_residual)},
        {"root_innovation", Vec3ToJson(contact.root_innovation)},
        {"correction_m", contact.correction_m},
        {"left_residual_m", contact.left_residual_m},
        {"right_residual_m", contact.right_residual_m},
        {"common_residual_m", contact.common_residual_m},
        {"root_innovation_m", contact.root_innovation_m},
        {"foot_disagreement_m", contact.disagreement_m},
        {"root_alignment", contact.root_alignment}
    };
    return out;
}

nlohmann::json TrackerEkfRoleToJson(const TrackerEkfRoleTelemetry& role) {
    return {
        {"initialized", role.initialized},
        {"filtered", role.filtered},
        {"locked_reset", role.locked_reset},
        {"support_confidence", role.support_confidence},
        {"measurement_variance_m2", role.measurement_variance_m2},
        {"innovation_m", role.innovation_m},
        {"mean_position_gain", role.mean_position_gain},
        {"orientation_gain", role.orientation_gain}
    };
}

nlohmann::json TrackerEkfToJson(const TrackerEkfTelemetry& telemetry) {
    return {
        {"enabled", telemetry.enabled},
        {"applied", telemetry.applied},
        {"reset", telemetry.reset},
        {"input_confidence", telemetry.input_confidence},
        {"root", TrackerEkfRoleToJson(telemetry.root)},
        {"left_foot", TrackerEkfRoleToJson(telemetry.left_foot)},
        {"right_foot", TrackerEkfRoleToJson(telemetry.right_foot)},
        {"root_initialized", telemetry.root_initialized},
        {"left_foot_initialized", telemetry.left_foot_initialized},
        {"right_foot_initialized", telemetry.right_foot_initialized}
    };
}

nlohmann::json BodyStateJointToJson(const BodyStateJoint& joint) {
    return {
        {"role", ToString(joint.role)},
        {"valid", joint.valid},
        {"position", Vec3ToJson(joint.position)},
        {"velocity", Vec3ToJson(joint.velocity)},
        {"confidence", joint.confidence},
        {"visibility", ToString(joint.visibility)},
        {"evidence", TrackerEvidenceToJson(joint.evidence)},
        {"depth_source", ToString(joint.depth_source)},
        {"measured", joint.measured},
        {"predicted", joint.predicted},
        {"camera_a_present", joint.camera_a_present},
        {"camera_b_present", joint.camera_b_present},
        {"camera_a_confidence", joint.camera_a_confidence},
        {"camera_b_confidence", joint.camera_b_confidence},
        {"camera_a_weight", joint.camera_a_weight},
        {"camera_b_weight", joint.camera_b_weight},
        {"camera_a_quality", joint.camera_a_quality},
        {"camera_b_quality", joint.camera_b_quality},
        {"evidence_source", ToString(joint.evidence_source)},
        {"triangulated", joint.triangulated},
        {"depth_inferred", joint.depth_inferred},
        {"reprojection_error_px", joint.reprojection_error_px},
        {"estimated_depth_m", joint.estimated_depth_m},
        {"contact_lock_strength", joint.contact_lock_strength},
        {"contact_support_confidence", joint.contact_support_confidence},
        {"solver_observation_weighted", joint.solver_observation_weighted},
        {"solver_observation_weight_scale", joint.solver_observation_weight_scale},
        {"solver_observation_confidence_ceiling", joint.solver_observation_confidence_ceiling},
        {"identity_confidence", joint.identity_confidence},
        {"reason", joint.reason}
    };
}

nlohmann::json BodyStateToJson(const UnifiedBodyState& state) {
    auto roles = nlohmann::json::array();
    for (const auto role : kBodyJointRoles) {
        roles.push_back(BodyStateJointToJson(state.roles[BodyJointRoleIndex(role)]));
    }
    return {
        {"valid", state.valid},
        {"left_foot_contact", ToString(state.left_foot_contact)},
        {"right_foot_contact", ToString(state.right_foot_contact)},
        {"diagnostics", {
            {"active", state.diagnostics.active},
            {"degraded", state.diagnostics.degraded},
            {"triangulation_active", state.diagnostics.triangulation_active},
            {"tracking_mode_is_monocular", state.diagnostics.tracking_mode_is_monocular},
            {"stereo_fallback_active", state.diagnostics.stereo_fallback_active},
            {"monocular_fallback", state.diagnostics.monocular_fallback},
            {"left_right_identity_stable", state.diagnostics.left_right_identity_stable},
            {"left_right_identity_uncertain", state.diagnostics.left_right_identity_uncertain},
            {"occlusion_prediction_active", state.diagnostics.occlusion_prediction_active},
            {"contact_lock_active", state.diagnostics.contact_lock_active},
            {"floor_support_active", state.diagnostics.floor_support_active},
            {"body_calibration_valid", state.diagnostics.body_calibration_valid},
            {"latency_prediction_active", state.diagnostics.latency_prediction_active},
            {"latency_prediction_seconds", state.diagnostics.latency_prediction_seconds},
            {"triangulated_count", state.diagnostics.triangulated_count},
            {"inferred_depth_count", state.diagnostics.inferred_depth_count},
            {"predicted_joint_count", state.diagnostics.predicted_joint_count},
            {"measured_role_count", state.diagnostics.measured_role_count},
            {"low_confidence_role_count", state.diagnostics.low_confidence_role_count},
            {"mean_reprojection_error_px", state.diagnostics.mean_reprojection_error_px},
            {"role_output_confidence", state.diagnostics.role_output_confidence},
            {"identity_confidence", state.diagnostics.identity_confidence},
            {"left_contact_lock_strength", state.diagnostics.left_contact_lock_strength},
            {"right_contact_lock_strength", state.diagnostics.right_contact_lock_strength},
            {"tracking_mode", state.diagnostics.tracking_mode},
            {"depth_source", state.diagnostics.depth_source},
            {"reason", state.diagnostics.reason}
        }},
        {"roles", roles}
    };
}

nlohmann::json DepthTelemetryToJson(
    const TrackingSolverTelemetry& solver,
    const FloorGeometryCalibration& floor_geometry) {
    const auto& stereo = solver.preliminary_stereo;
    const bool accepted = AcceptedRuntimeFloorGeometry(floor_geometry);
    const std::string used_source = ToString(stereo.monocular_scale_source);
    const std::string accepted_source = AcceptedFloorGeometrySource(floor_geometry);
    return {
        {"source", ToString(solver.depth_source)},
        {"confidence", stereo.mean_confidence},
        {"foot_confidence", stereo.foot_mean_confidence},
        {"camera_a_quality", stereo.camera_a_mean_quality},
        {"camera_b_quality", stereo.camera_b_mean_quality},
        {"camera_a_present_keypoints", stereo.camera_a_present_keypoints},
        {"camera_b_present_keypoints", stereo.camera_b_present_keypoints},
        {"camera_a_usable_keypoints", stereo.camera_a_usable_keypoints},
        {"camera_b_usable_keypoints", stereo.camera_b_usable_keypoints},
        {"camera_a_age_scale", stereo.camera_a_age_scale},
        {"camera_b_age_scale", stereo.camera_b_age_scale},
        {"inferred_count", stereo.inferred_depth_count},
        {"triangulated_count", stereo.triangulated_count},
        {"mean_inferred_depth_m", stereo.mean_inferred_depth_m},
        {"scale_source", ToString(stereo.monocular_scale_source)},
        {"floor_assist", {
            {"status", MonocularFloorAssistStatus(stereo, floor_geometry)},
            {"source", used_source != "none" ? used_source : accepted_source},
            {"used_source", used_source},
            {"accepted_source", accepted_source},
            {"depth_m", stereo.monocular_floor_assist_depth_m},
            {"confidence", accepted ? std::max(stereo.monocular_floor_assist_confidence, floor_geometry.metric_scale_confidence)
                                   : stereo.monocular_floor_assist_confidence},
            {"floor_geometry_accepted", accepted},
            {"floor_geometry_projective_accepted", AcceptedRuntimeProjectiveFloorGeometry(floor_geometry)},
            {"distortion_correction_used", stereo.floor_distortion_correction_used},
            {"camera_orientation_used", stereo.floor_camera_orientation_used}
        }}
    };
}

nlohmann::json BodySolveJointTriangulationToJson(
    const BodySolveJointTriangulationTelemetry& joint,
    std::size_t index) {
    const auto id = static_cast<KeypointId>(index);
    return {
        {"id", index},
        {"name", ToString(id)},
        {"camera_a_present", joint.camera_a_present},
        {"camera_b_present", joint.camera_b_present},
        {"camera_a_confidence", joint.camera_a_confidence},
        {"camera_b_confidence", joint.camera_b_confidence},
        {"camera_a_weight", joint.camera_a_weight},
        {"camera_b_weight", joint.camera_b_weight},
        {"camera_a_quality", joint.camera_a_quality},
        {"camera_b_quality", joint.camera_b_quality},
        {"temporal_confidence", joint.temporal_confidence},
        {"epipolar_available", joint.epipolar_available},
        {"epipolar_checked", joint.epipolar_checked},
        {"epipolar_error_px", joint.epipolar_error_px},
        {"epipolar_error_px_isotropic_heuristic", joint.epipolar_error_px_isotropic},
        {"epipolar_error_px_anisotropic", joint.epipolar_error_px_anisotropic},
        {"epipolar_error_normalized", joint.epipolar_error_normalized},
        {"epipolar_confidence", joint.epipolar_confidence},
        {"epipolar_reliability_term", joint.epipolar_reliability_term},
        {"epipolar_hard_mismatch", joint.epipolar_hard_mismatch},
        {"epipolar_pair_rejected", joint.epipolar_pair_rejected},
        {"epipolar_degraded_pair_softened", joint.epipolar_degraded_pair_softened},
        {"epipolar_reason", ToString(joint.epipolar_reason)},
        {"epipolar_coordinate_space", ToString(joint.epipolar_coordinate_space)},
        {"used_temporal_depth", joint.used_temporal_depth},
        {"fallback_used", joint.fallback_used},
        {"evidence_source", ToString(joint.evidence_source)},
        {"triangulated", joint.triangulated},
        {"depth_inferred", joint.depth_inferred},
        {"depth_source", ToString(joint.depth_source)},
        {"world", Vec3ToJson(joint.world)},
        {"anchor_raw_world_present", joint.anchor_raw_world_present},
        {"anchor_raw_world", Vec3ToJson(joint.anchor_raw_world)},
        {"anchor_correction_applied", joint.anchor_correction_applied},
        {"anchor_corrected_world", Vec3ToJson(joint.anchor_corrected_world)},
        {"anchor_corrected_depth_m", joint.anchor_corrected_depth_m},
        {"anchor_correction_rejection_reason", joint.anchor_correction_rejection_reason},
        {"confidence", joint.confidence},
        {"reprojection_error_a_px", joint.reprojection_error_a_px},
        {"reprojection_error_b_px", joint.reprojection_error_b_px},
        {"mean_reprojection_error_px", joint.mean_reprojection_error_px},
        {"triangulation_condition_number", joint.triangulation_condition_number},
        {"triangulation_strength_ratio", joint.triangulation_strength_ratio},
        {"triangulation_null_residual", joint.triangulation_null_residual},
        {"measurement_uncertainty_valid", joint.measurement_uncertainty_valid},
        {"measurement_baseline_m", joint.measurement_baseline_m},
        {"measurement_mean_depth_m", joint.measurement_mean_depth_m},
        {"measurement_baseline_to_depth_ratio", joint.measurement_baseline_to_depth_ratio},
        {"measurement_effective_focal_px", joint.measurement_effective_focal_px},
        {"measurement_reprojection_sigma_px", joint.measurement_reprojection_sigma_px},
        {"measurement_epipolar_sigma_px", joint.measurement_epipolar_sigma_px},
        {"measurement_image_noise_sigma_px", joint.measurement_image_noise_sigma_px},
        {"measurement_conditioning_scale", joint.measurement_conditioning_scale},
        {"measurement_unclamped_lateral_stddev_m", joint.measurement_unclamped_lateral_stddev_m},
        {"measurement_unclamped_depth_stddev_m", joint.measurement_unclamped_depth_stddev_m},
        {"measurement_unclamped_position_variance_m2", joint.measurement_unclamped_position_variance_m2},
        {"measurement_lateral_stddev_m", joint.measurement_lateral_stddev_m},
        {"measurement_depth_stddev_m", joint.measurement_depth_stddev_m},
        {"measurement_position_stddev_m", joint.measurement_position_stddev_m},
        {"measurement_position_variance_m2", joint.measurement_position_variance_m2},
        {"solver_uncertainty_weighted", joint.solver_uncertainty_weighted},
        {"solver_uncertainty_valid", joint.solver_uncertainty_valid},
        {"solver_uncertainty_conservative_fallback", joint.solver_uncertainty_conservative_fallback},
        {"solver_temporal_process_noise_applied", joint.solver_temporal_process_noise_applied},
        {"solver_lateral_weight_scale", joint.solver_lateral_weight_scale},
        {"solver_depth_weight_scale", joint.solver_depth_weight_scale},
        {"solver_observation_confidence_ceiling", joint.solver_observation_confidence_ceiling},
        {"solver_temporal_process_stddev_m", joint.solver_temporal_process_stddev_m},
        {"estimated_depth_m", joint.estimated_depth_m},
        {"foot_contact_confidence", joint.foot_contact_confidence}
    };
}

nlohmann::json BodySolveStereoToJson(
    const BodySolveStereoTelemetry& stereo,
    const FloorGeometryCalibration& floor_geometry) {
    nlohmann::json joints = nlohmann::json::array();
    for (std::size_t i = 0; i < stereo.joints.size(); ++i) {
        joints.push_back(BodySolveJointTriangulationToJson(stereo.joints[i], i));
    }

    return {
        {"tracking_mode", ToString(stereo.tracking_mode)},
        {"depth_source", ToString(stereo.depth_source)},
        {"triangulated_count", stereo.triangulated_count},
        {"left_foot_triangulated_count", stereo.left_foot_triangulated_count},
        {"right_foot_triangulated_count", stereo.right_foot_triangulated_count},
        {"inferred_depth_count", stereo.inferred_depth_count},
        {"camera_a_present_keypoints", stereo.camera_a_present_keypoints},
        {"camera_b_present_keypoints", stereo.camera_b_present_keypoints},
        {"camera_a_usable_keypoints", stereo.camera_a_usable_keypoints},
        {"camera_b_usable_keypoints", stereo.camera_b_usable_keypoints},
        {"camera_a_mean_quality", stereo.camera_a_mean_quality},
        {"camera_b_mean_quality", stereo.camera_b_mean_quality},
        {"camera_a_age_scale", stereo.camera_a_age_scale},
        {"camera_b_age_scale", stereo.camera_b_age_scale},
        {"epipolar_geometry_valid", stereo.epipolar_geometry_valid},
        {"epipolar_status", stereo.epipolar_status},
        {"epipolar_checked_count", stereo.epipolar_checked_count},
        {"epipolar_hard_mismatch_count", stereo.epipolar_hard_mismatch_count},
        {"epipolar_pair_rejected_count", stereo.epipolar_pair_rejected_count},
        {"epipolar_degraded_pair_softened_count", stereo.epipolar_degraded_pair_softened_count},
        {"mean_epipolar_error_px", stereo.mean_epipolar_error_px},
        {"mean_epipolar_error_px_isotropic_heuristic", stereo.mean_epipolar_error_px_isotropic},
        {"mean_epipolar_error_px_anisotropic", stereo.mean_epipolar_error_px_anisotropic},
        {"mean_epipolar_error_normalized", stereo.mean_epipolar_error_normalized},
        {"mean_epipolar_confidence", stereo.mean_epipolar_confidence},
        {"mean_inferred_depth_m", stereo.mean_inferred_depth_m},
        {"mean_confidence", stereo.mean_confidence},
        {"foot_mean_confidence", stereo.foot_mean_confidence},
        {"mean_reprojection_error_px", stereo.mean_reprojection_error_px},
        {"mean_triangulation_condition_number", stereo.mean_triangulation_condition_number},
        {"mean_triangulation_strength_ratio", stereo.mean_triangulation_strength_ratio},
        {"mean_triangulation_null_residual", stereo.mean_triangulation_null_residual},
        {"measurement_uncertainty_count", stereo.measurement_uncertainty_count},
        {"mean_measurement_position_stddev_m", stereo.mean_measurement_position_stddev_m},
        {"mean_measurement_depth_stddev_m", stereo.mean_measurement_depth_stddev_m},
        {"mean_measurement_baseline_to_depth_ratio", stereo.mean_measurement_baseline_to_depth_ratio},
        {"solver_uncertainty_weighted_count", stereo.solver_uncertainty_weighted_count},
        {"solver_uncertainty_valid_count", stereo.solver_uncertainty_valid_count},
        {"solver_uncertainty_conservative_fallback_count", stereo.solver_uncertainty_conservative_fallback_count},
        {"solver_temporal_process_noise_count", stereo.solver_temporal_process_noise_count},
        {"mean_solver_lateral_weight_scale", stereo.mean_solver_lateral_weight_scale},
        {"mean_solver_depth_weight_scale", stereo.mean_solver_depth_weight_scale},
        {"mean_solver_observation_confidence_ceiling", stereo.mean_solver_observation_confidence_ceiling},
        {"mean_solver_temporal_process_stddev_m", stereo.mean_solver_temporal_process_stddev_m},
        {"foot_mean_reprojection_error_px", stereo.foot_mean_reprojection_error_px},
        {"max_foot_reprojection_error_px", stereo.max_foot_reprojection_error_px},
        {"camera_a_geometry_used", stereo.camera_a_geometry_used},
        {"camera_b_geometry_used", stereo.camera_b_geometry_used},
        {"stereo_geometry_constraints_used", stereo.stereo_geometry_constraints_used},
        {"stereo_geometry_confidence", stereo.stereo_geometry_confidence},
        {"geometry_stereo_status", stereo.geometry_stereo_status},
        {"hmd_depth_scale", HmdDepthScaleToJson(stereo.hmd_depth_scale)},
        {"anchor_space_mapping", ProjectionCorrectionToJson(stereo.anchor_space_mapping)},
        {"room_depth_map", RoomDepthMapToJson(stereo.room_depth_map)},
        {"left_foot_contact_confidence", stereo.left_foot_contact_confidence},
        {"right_foot_contact_confidence", stereo.right_foot_contact_confidence},
        {"left_heel_contact_confidence", stereo.left_heel_contact_confidence},
        {"left_toe_contact_confidence", stereo.left_toe_contact_confidence},
        {"right_heel_contact_confidence", stereo.right_heel_contact_confidence},
        {"right_toe_contact_confidence", stereo.right_toe_contact_confidence},
        {"left_knee_floor_contact_confidence", stereo.left_knee_floor_contact_confidence},
        {"right_knee_floor_contact_confidence", stereo.right_knee_floor_contact_confidence},
        {"left_knee_floor_contact_observed", stereo.left_knee_floor_contact_observed},
        {"right_knee_floor_contact_observed", stereo.right_knee_floor_contact_observed},
        {"left_foot_low_res_separation_px", stereo.left_foot_low_res_separation_px},
        {"right_foot_low_res_separation_px", stereo.right_foot_low_res_separation_px},
        {"monocular_scale_source", ToString(stereo.monocular_scale_source)},
        {"monocular_floor_assist_status", MonocularFloorAssistStatus(stereo, floor_geometry)},
        {"monocular_floor_assist_source", ToString(stereo.monocular_scale_source) != std::string("none")
            ? ToString(stereo.monocular_scale_source)
            : AcceptedFloorGeometrySource(floor_geometry)},
        {"monocular_floor_assist_depth_m", stereo.monocular_floor_assist_depth_m},
        {"monocular_floor_assist_confidence", stereo.monocular_floor_assist_confidence},
        {"floor_geometry", {
            {"accepted", AcceptedRuntimeFloorGeometry(floor_geometry)},
            {"projective_accepted", AcceptedRuntimeProjectiveFloorGeometry(floor_geometry)},
            {"used", stereo.floor_geometry_used},
            {"source", stereo.floor_geometry_used ? FloorGeometrySource(floor_geometry) : AcceptedFloorGeometrySource(floor_geometry)},
            {"confidence", AcceptedRuntimeFloorGeometry(floor_geometry)
                ? std::max(stereo.floor_geometry_confidence, floor_geometry.metric_scale_confidence)
                : stereo.floor_geometry_confidence},
            {"family_count", AcceptedRuntimeFloorGeometry(floor_geometry)
                ? floor_geometry.family_count
                : stereo.floor_geometry_family_count},
            {"distortion_correction_used", stereo.floor_distortion_correction_used},
            {"camera_orientation_used", stereo.floor_camera_orientation_used}
        }},
        {"joints", joints}
    };
}

} // namespace

Status ReplayLogWriter::Open(const std::filesystem::path& path) {
    return Open(path, ReplayLogSessionInfo{});
}

Status ReplayLogWriter::Open(const std::filesystem::path& path, const ReplayLogSessionInfo& session) {
    Close();
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    path_ = path;
    session_ = session;
    if (session_.schema_version <= 0) {
        session_.schema_version = 3;
    }
    frame_index_ = 0;
    out_.open(path_, std::ios::out | std::ios::trunc);
    if (!out_) {
        return Status::Error(StatusCode::InvalidArgument, "Could not open replay log");
    }
    open_ = true;
    out_ << ReplayManifestToJson(session_).dump() << '\n';
    if (!out_) {
        Close();
        return Status::Error(StatusCode::InternalError, "Replay manifest write failed");
    }
    return Status::OK();
}

Status ReplayLogWriter::WriteSnapshot(const DebugSnapshot& snapshot) {
    if (!open_ || !out_.is_open()) {
        return Status::Error(StatusCode::FailedPrecondition, "Replay log is not open");
    }

    const auto solver = EffectiveSolverTelemetry(snapshot);
    const auto& trackers = EffectiveTrackers(snapshot);
    const Pose3f root_pose = EffectivePose(snapshot, TrackerRole::Pelvis, snapshot.tracking.state.root);
    const Pose3f left_foot_pose = EffectivePose(snapshot, TrackerRole::LeftFoot, snapshot.tracking.state.left_foot);
    const Pose3f right_foot_pose = EffectivePose(snapshot, TrackerRole::RightFoot, snapshot.tracking.state.right_foot);

    nlohmann::json j = {
        {"type", "replay_frame"},
        {"schema_version", session_.schema_version},
        {"frame_index", frame_index_++},
        {"deterministic", session_.deterministic},
        {"phase", snapshot.phase},
        {"timestamp_seconds", snapshot.timestamp_seconds != 0.0 ? snapshot.timestamp_seconds : snapshot.time_seconds},
        {"degradation_mode", snapshot.degradation_mode.empty() ? snapshot.tracking.degradation_mode : snapshot.degradation_mode},
        {"last_error", snapshot.last_error.empty() ? snapshot.tracking.last_error : snapshot.last_error},
        {"camera_a", {
            {"opened", snapshot.camera_a.opened},
            {"running", snapshot.camera_a.running},
            {"delivered", snapshot.camera_a.delivered_frames},
            {"read_failures", snapshot.camera_a.read_failures},
            {"width", snapshot.camera_a.actual_width},
            {"height", snapshot.camera_a.actual_height}
        }},
        {"camera_b", {
            {"opened", snapshot.camera_b.opened},
            {"running", snapshot.camera_b.running},
            {"delivered", snapshot.camera_b.delivered_frames},
            {"read_failures", snapshot.camera_b.read_failures},
            {"width", snapshot.camera_b.actual_width},
            {"height", snapshot.camera_b.actual_height}
        }},
        {"frame_ages_ms", {
            {"camera_a", snapshot.camera_a_frame_age_ms},
            {"camera_b", snapshot.camera_b_frame_age_ms}
        }},
        {"frame_pairing", {
            {"accepted_pairs", snapshot.frame_pairing.accepted_pairs},
            {"missing_a", snapshot.frame_pairing.missing_a},
            {"missing_b", snapshot.frame_pairing.missing_b},
            {"rejected_skew", snapshot.frame_pairing.rejected_skew},
            {"rejected_duplicate", snapshot.frame_pairing.rejected_duplicate},
            {"rejected_reused_a", snapshot.frame_pairing.rejected_reused_a},
            {"rejected_reused_b", snapshot.frame_pairing.rejected_reused_b},
            {"degraded_skew", snapshot.frame_pairing.degraded_skew},
            {"degraded_duplicate", snapshot.frame_pairing.degraded_duplicate},
            {"degraded_reused_a", snapshot.frame_pairing.degraded_reused_a},
            {"degraded_reused_b", snapshot.frame_pairing.degraded_reused_b},
            {"last_accepted_sequence_a", snapshot.frame_pairing.last_accepted_sequence_a != 0 ? snapshot.frame_pairing.last_accepted_sequence_a : snapshot.frame_a_sequence},
            {"last_accepted_sequence_b", snapshot.frame_pairing.last_accepted_sequence_b != 0 ? snapshot.frame_pairing.last_accepted_sequence_b : snapshot.frame_b_sequence},
            {"last_skew_ms", snapshot.frame_pairing.last_skew_ms != 0.0 ? snapshot.frame_pairing.last_skew_ms : snapshot.frame_skew_ms},
            {"current_degraded", snapshot.frame_pair_degraded},
            {"current_reused_a", snapshot.frame_pair_reused_a},
            {"current_reused_b", snapshot.frame_pair_reused_b},
            {"current_duplicate", snapshot.frame_pair_duplicate},
            {"current_skewed", snapshot.frame_pair_skewed},
            {"current_reason", snapshot.frame_pair_reason}
        }},
        {"camera_a_roi", RoiToJson(snapshot.camera_a_roi.initialized ? snapshot.camera_a_roi : snapshot.view_a_roi)},
        {"camera_b_roi", RoiToJson(snapshot.camera_b_roi.initialized ? snapshot.camera_b_roi : snapshot.view_b_roi)},
        {"camera_a_pose_confidence", snapshot.camera_a_pose_confidence},
        {"camera_b_pose_confidence", snapshot.camera_b_pose_confidence},
        {"camera_a_reliability", snapshot.camera_a_reliability},
        {"camera_b_reliability", snapshot.camera_b_reliability},
        {"camera_a_pose", Pose2dToJson(snapshot.camera_a_pose)},
        {"camera_b_pose", Pose2dToJson(snapshot.camera_b_pose)},
        {"camera_a_reliability_full", ReliabilityToJson(snapshot.camera_a_reliability_full.mean_weight != 0.0f ? snapshot.camera_a_reliability_full : snapshot.view_a_reliability)},
        {"camera_b_reliability_full", ReliabilityToJson(snapshot.camera_b_reliability_full.mean_weight != 0.0f ? snapshot.camera_b_reliability_full : snapshot.view_b_reliability)},
        {"timing", {
            {"capture_ms", snapshot.capture_ms},
            {"frame_pair_ms", snapshot.frame_pair_ms},
            {"preprocess_ms", snapshot.preprocess_ms},
            {"preprocess_ms_a", snapshot.preprocess_ms_a},
            {"preprocess_ms_b", snapshot.preprocess_ms_b},
            {"onnx_ms", snapshot.onnx_ms},
            {"onnx_ms_a", snapshot.onnx_ms_a},
            {"onnx_ms_b", snapshot.onnx_ms_b},
            {"decode_ms", snapshot.decode_ms},
            {"decode_ms_a", snapshot.decode_ms_a},
            {"decode_ms_b", snapshot.decode_ms_b},
            {"inference_ms", snapshot.inference_ms != 0.0 ? snapshot.inference_ms : snapshot.inference_ms_a + snapshot.inference_ms_b},
            {"inference_ms_a", snapshot.inference_ms_a},
            {"inference_ms_b", snapshot.inference_ms_b},
            {"pipeline_ms", snapshot.pipeline_ms},
            {"solver_ms", snapshot.solver_ms},
            {"preliminary_solve_ms", snapshot.preliminary_solve_ms},
            {"final_solve_ms", snapshot.final_solve_ms},
            {"osc_ms", snapshot.osc_ms},
            {"ui_publish_ms", snapshot.ui_publish_ms},
            {"total_ms", snapshot.total_ms}
        }},
        {"osc", OscToJson(snapshot)},
        {"steamvr_alignment", SteamVrAlignmentToJson(snapshot)},
        {"hmd", {
            {"valid", snapshot.hmd_valid || snapshot.hmd.valid},
            {"timestamp_seconds", snapshot.hmd.timestamp_seconds},
            {"pose", PoseToJson(snapshot.hmd.pose)}
        }},
        {"hmd_depth_scale", HmdDepthScaleToJson(solver.hmd_depth_scale)},
        {"stereo_hmd_anchor", StereoHmdAnchorToJson(solver.stereo_hmd_anchor)},
        {"anchor_space_mapping", ProjectionCorrectionToJson(solver.anchor_space_mapping)},
        {"room_depth_map", RoomDepthMapToJson(solver.room_depth_map)},
        {"tracking", {
            {"posture_mode", ToString(snapshot.tracking.state.posture_mode)},
            {"confidence", EffectiveTrackingConfidence(snapshot)},
            {"root", PoseToJson(root_pose)},
            {"left_foot", PoseToJson(left_foot_pose)},
            {"right_foot", PoseToJson(right_foot_pose)},
            {"leg_dofs", LegDofsToJson(snapshot.tracking.state)},
            {"body_calibration", BodyCalibrationTelemetryToJson(snapshot.tracking.body_calibration)},
            {"floor_geometry", FloorGeometryToJson(snapshot.tracking.floor_geometry)},
            {"left_foot_support", ToString(snapshot.tracking.state.support.left_foot.type)},
            {"left_foot_phase", ToString(snapshot.tracking.state.support.left_foot.phase)},
            {"right_foot_support", ToString(snapshot.tracking.state.support.right_foot.type)},
            {"right_foot_phase", ToString(snapshot.tracking.state.support.right_foot.phase)},
            {"root_support", ToString(snapshot.tracking.state.support.root_support)},
            {"support", {
                {"root", {
                    {"type", ToString(snapshot.tracking.state.support.root_support)},
                    {"anchor", AnchorToJson(snapshot.tracking.state.support.root_anchor)},
                    {"left_knee_anchor", AnchorToJson(snapshot.tracking.state.support.left_knee_anchor)},
                    {"right_knee_anchor", AnchorToJson(snapshot.tracking.state.support.right_knee_anchor)}
                }},
                {"left_foot", FootSupportToJson(snapshot.tracking.state.support.left_foot, snapshot.tracking.state.left_foot, snapshot.tracking.body_calibration.body.left_foot_length)},
                {"right_foot", FootSupportToJson(snapshot.tracking.state.support.right_foot, snapshot.tracking.state.right_foot, snapshot.tracking.body_calibration.body.right_foot_length)}
            }},
            {"stages", TrackingStagesToJson(snapshot.tracking.stages)},
            {"body_state", BodyStateToJson(snapshot.tracking.body_state)},
            {"motion_filter", MotionFilterToJson(snapshot.tracking.motion_filter)},
            {"tracker_ekf", TrackerEkfToJson(snapshot.tracking.tracker_ekf)},
            {"solver", {
                {"tracking_mode", ToString(solver.tracking_mode)},
                {"depth_source", ToString(solver.depth_source)},
                {"depth", DepthTelemetryToJson(solver, snapshot.tracking.floor_geometry)},
                {"used_hmd", solver.used_hmd},
                {"degraded", solver.degraded},
                {"reason", solver.reason},
                {"camera_a_identity_swapped", solver.camera_a_identity_swapped},
                {"camera_b_identity_swapped", solver.camera_b_identity_swapped},
                {"camera_a_identity_consistency", solver.camera_a_identity_consistency},
                {"camera_b_identity_consistency", solver.camera_b_identity_consistency},
                {"identity_epipolar_arbitration_checked", solver.identity_epipolar_arbitration_checked},
                {"identity_epipolar_arbitration_applied", solver.identity_epipolar_arbitration_applied},
                {"identity_epipolar_scored_lateral_pairs", solver.identity_epipolar_scored_lateral_pairs},
                {"identity_epipolar_same_score", solver.identity_epipolar_same_score},
                {"identity_epipolar_cross_score", solver.identity_epipolar_cross_score},
                {"identity_epipolar_cross_geometric_uncertainty", solver.identity_epipolar_cross_geometric_uncertainty},
                {"identity_epipolar_detection_support", solver.identity_epipolar_detection_support},
                {"identity_epipolar_required_swap_margin", solver.identity_epipolar_required_swap_margin},
                {"identity_same_mahalanobis_sq", solver.identity_same_mahalanobis_sq},
                {"identity_cross_mahalanobis_sq", solver.identity_cross_mahalanobis_sq},
                {"identity_same_negative_log_likelihood", solver.identity_same_negative_log_likelihood},
                {"identity_cross_negative_log_likelihood", solver.identity_cross_negative_log_likelihood},
                {"identity_cross_within_mahalanobis_gate", solver.identity_cross_within_mahalanobis_gate},
                {"identity_score_gate_passed", solver.identity_score_gate_passed},
                {"identity_likelihood_gate_passed", solver.identity_likelihood_gate_passed},
                {"identity_swap_blocked_by_strong_consistency", solver.identity_swap_blocked_by_strong_consistency},
                {"identity_swap_blocked_by_tie", solver.identity_swap_blocked_by_tie},
                {"identity_uncertainty_fallback_count", solver.identity_uncertainty_fallback_count},
                {"preliminary_solve_ms", solver.preliminary_solve_ms},
                {"final_solve_ms", solver.final_solve_ms},
                {"preliminary_residual", solver.preliminary_residual},
                {"final_residual", solver.final_residual},
                {"preliminary_weighted_observation_count", solver.preliminary_weighted_observation_count},
                {"final_weighted_observation_count", solver.final_weighted_observation_count},
                {"support_constraints", SupportSolveConstraintsToJson(solver.final_constraints)},
                {"triangulation", {
                    {"preliminary", BodySolveStereoToJson(
                        solver.preliminary_stereo,
                        snapshot.tracking.floor_geometry)}
                }},
                {"objective_evaluations", solver.objective_evaluations},
                {"coordinate_passes", solver.coordinate_passes},
                {"optimizer_early_stopped", solver.optimizer_early_stopped}
            }},
            {"trackers", TrackersToJson(trackers)}
        }}
    };
    out_ << j.dump() << '\n';
    if (!out_) {
        return Status::Error(StatusCode::InternalError, "Replay log write failed");
    }
    return Status::OK();
}

Status ReplayLogWriter::Append(const DebugSnapshot& snapshot) {
    return WriteSnapshot(snapshot);
}

void ReplayLogWriter::Close() {
    if (out_.is_open()) {
        out_.flush();
        out_.close();
    }
    open_ = false;
}

} // namespace bt
