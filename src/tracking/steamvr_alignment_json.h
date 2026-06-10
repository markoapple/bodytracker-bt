#pragma once

#include "tracking/steamvr_alignment.h"

#include <nlohmann/json.hpp>

namespace bt {

// Single source of truth for alignment status JSON. Used by:
//   * desktop UI getStateJson()
//   * DebugSnapshot exporter
//   * replay/debug log exporter
//
// Keeping one serializer means UI strings, debug strings, and replay strings
// can never disagree on what the backend says. UI must NEVER author its own
// alignment math; it only reads this struct.
inline nlohmann::json Vec3ToJsonArray(const Vec3f& v) {
    return nlohmann::json::array({v.x, v.y, v.z});
}

inline nlohmann::json QuatToJsonArray(const Quatf& q) {
    return nlohmann::json::array({q.x, q.y, q.z, q.w});
}

inline nlohmann::json AlignmentSampleToJson(const SteamVrAlignmentSample& sample) {
    return {
        {"landmark", ToString(sample.landmark)},
        {"landmark_key", LandmarkKey(sample.landmark)},
        {"controller", ToString(sample.controller)},
        {"controller_valid", sample.controller_valid},
        {"body_state_valid", sample.body_state_valid},
        {"accepted", sample.accepted},
        {"confidence", sample.confidence},
        {"residual_m", sample.residual_m},
        {"timestamp_seconds", sample.timestamp_seconds},
        {"pose_age_seconds", sample.pose_age_seconds},
        {"reason", sample.reason},
        {"reason_code", ToString(sample.reason_code)},
        {"steamvr_position", Vec3ToJsonArray(sample.steamvr_pose.position)},
        {"camera_landmark", Vec3ToJsonArray(sample.camera_landmark)},
    };
}

inline nlohmann::json AlignmentStatusToJson(const SteamVrAlignmentStatus& s) {
    nlohmann::json samples = nlohmann::json::array();
    for (const auto& sample : s.samples) {
        samples.push_back(AlignmentSampleToJson(sample));
    }
    return {
        {"provider", {
            {"available", s.provider_available},
            {"runtime_initialized", s.provider_runtime_initialized},
            {"status", s.provider_status},
            {"reason", s.provider_reason},
            {"controller_device_count", s.controller_device_count},
            {"left_controller_tracked", s.left_controller_tracked},
            {"right_controller_tracked", s.right_controller_tracked},
            {"left_trigger_pressed", s.left_trigger_pressed},
            {"right_trigger_pressed", s.right_trigger_pressed},
            {"left_trigger_pressed_edge", s.left_trigger_pressed_edge},
            {"right_trigger_pressed_edge", s.right_trigger_pressed_edge},
            {"last_pose_age_seconds", s.last_pose_age_seconds},
            {"max_allowed_pose_age_seconds", s.max_allowed_pose_age_seconds},
            {"hard_unavailable", s.provider_hard_unavailable},
            {"compile_disabled", s.provider_compile_disabled},
            {"controller_alignment_fresh", s.controller_alignment_fresh},
        }},
        {"session", {
            {"active", s.session_active},
            {"accepted_sample_count", s.accepted_sample_count},
            {"total_samples_recorded", s.total_samples_recorded},
            {"required_samples_present", s.required_samples_present},
            {"required_samples_complete", s.required_samples_complete},
        }},
        {"transform", {
            {"valid", s.transform_valid},
            {"stale", s.stale},
            {"role_offsets_present", s.role_offsets_present},
            {"source_known", s.source_known},
            {"raw_active_transform_source", s.raw_active_transform_source},
            {"state", s.state},
            {"reason", s.reason},
            {"reason_code", ToString(s.reason_code)},
            {"stale_reason", s.stale ? (s.stale_reason.empty() ? s.reason : s.stale_reason) : ""},
            {"confidence", s.confidence},
            {"residual_m", s.residual_m},
            {"floor_residual_m", s.floor_residual_m},
            {"yaw_offset_rad", s.yaw_offset_rad},
            {"yaw_disagreement_rad", s.yaw_disagreement_rad},
            {"scale_ratio", s.scale_ratio},
            {"scale_mismatch", s.scale_mismatch},
            {"last_alignment_timestamp", s.last_alignment_timestamp},
        }},
        {"active_transform", {
            {"source", ToString(s.active_transform_source)},
            {"valid", s.transform_valid || s.manual_fallback_active},
            {"manual_fallback_available", s.manual_fallback_available},
            {"manual_fallback_active", s.manual_fallback_active},
            {"raw_source", s.raw_active_transform_source},
            {"source_known", s.source_known},
        }},
        {"context", {
            {"body_calibration_valid", s.body_calibration_valid},
            {"floor_calibration_valid", s.floor_calibration_valid},
            {"body_state_stable", s.body_state_stable},
            {"body_signature", s.body_signature},
            {"floor_signature", s.floor_signature},
        }},
        {"samples", samples},
    };
}

inline nlohmann::json LatencyProbeToJson(const LatencyProbeSession& probe) {
    return {
        {"status", ToString(probe.status)},
        {"reason", probe.reason},
        {"sample_count", probe.sample_count},
        {"estimated_latency_seconds", probe.estimated_latency_seconds},
        {"confidence", probe.confidence},
    };
}

} // namespace bt
