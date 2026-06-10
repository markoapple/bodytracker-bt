#pragma once
// depth_provenance.h
// Live trace-emission helpers called from the runtime pipeline.
// Each function fills one DepthTraceRecord for a specific stage boundary.
// All functions are header-only so they compile into any TU that includes them.

#include "debug/depth_trace.h"
#include "core/types.h"
#include "inference/rtmpose_decode.h"
#include "tracking/body_state.h"
#include "tracking/body_solver.h"
#include "tracking/tracker_synthesis.h"
#include "io/osc_sender.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace bt {

// ── Stage 1: raw RTMW3D model output ─────────────────────────────────────────

inline DepthTraceRecord TraceModelOutput(
    std::uint64_t frame_id,
    const ModelDepthKeypoint& kp,
    float confidence_xy)        // 2D confidence if z not available
{
    DepthTraceRecord r;
    r.frame_id         = frame_id;
    r.stage            = DepthTraceStage::ModelOutput;
    r.predecessor      = DepthTraceStage::ModelOutput;
    r.coordinate_frame = "model_simcc_body_relative";
    r.source_note      = "outputs[2] RTMW3D z-SimCC bins";
    r.model_z_decoded  = kp.z_decoded;
    r.model_raw_z      = kp.raw_z_bin;
    r.x = 0.0f; r.y = 0.0f;   // image x/y not recorded at this boundary
    r.z = kp.z_decoded ? kp.refined_z : 0.0f;
    r.xyz_valid   = kp.z_decoded;
    r.confidence  = kp.z_decoded ? kp.confidence_3d : confidence_xy;
    if (!kp.z_decoded) {
        r.z_state            = ZState::Absent;
        r.reason_if_no_value = "z_simcc_tensor_absent_or_not_rtmw3d_model";
    } else if (!std::isfinite(kp.refined_z)) {
        r.z_state = ZState::Invalid;
    } else if (kp.refined_z == 0.0f) {
        r.z_state = ZState::Zeroed;
    } else {
        r.z_state = ZState::Present;
    }
    return r;
}

// ── Stage 2: decoded keypoint ─────────────────────────────────────────────────

inline DepthTraceRecord TraceDecodedKeypoint(
    std::uint64_t frame_id,
    const Keypoint2D& kp2d,
    const ModelDepthKeypoint& kp3d)
{
    DepthTraceRecord r;
    r.frame_id         = frame_id;
    r.stage            = DepthTraceStage::DecodedKeypoint;
    r.predecessor      = DepthTraceStage::ModelOutput;
    // x/y are image pixels; z is a separate body-relative axis — never metric world
    r.coordinate_frame = "image_pixels_plus_model_body_relative_z";
    r.source_note      = "DecodedPose2D::keypoints[LeftAnkle]+DecodedPose3D::model_depth[LeftAnkle]";
    r.x = kp2d.pixel.x; r.y = kp2d.pixel.y;
    r.z = kp3d.z_decoded ? kp3d.refined_z : 0.0f;
    r.xyz_valid        = kp2d.present;
    r.confidence       = kp2d.confidence;
    r.model_z_decoded  = kp3d.z_decoded;
    r.model_raw_z      = kp3d.raw_z_bin;
    r.measurement_driven = kp2d.present;
    if (!kp3d.z_decoded) {
        r.z_state            = ZState::Absent;
        r.reason_if_no_value = "DecodedPose3D.valid=false";
        r.transform_note     = "image_xy_and_model_z_are_different_axes;not_metric_world";
    } else if (!std::isfinite(kp3d.refined_z)) {
        r.z_state = ZState::Invalid;
    } else if (kp3d.refined_z == 0.0f) {
        r.z_state = ZState::Zeroed;
    } else {
        r.z_state        = ZState::Present;
        r.transform_note = "z_is_model_body_relative_simcc_bin;NOT_metric_world_z";
    }
    return r;
}

// ── Stage 3: body solver input ────────────────────────────────────────────────

inline DepthTraceRecord TraceBodySolverInput(
    std::uint64_t frame_id,
    const BodySolveInputs& inputs)
{
    constexpr std::size_t kLA = static_cast<std::size_t>(KeypointId::LeftAnkle);
    DepthTraceRecord r;
    r.frame_id         = frame_id;
    r.stage            = DepthTraceStage::BodySolverInput;
    r.predecessor      = DepthTraceStage::DecodedKeypoint;
    r.coordinate_frame = "image_pixels_plus_model_body_relative_z";
    r.source_note      = "BodySolveInputs::camera_a_pose[LeftAnkle]+camera_a_pose_3d[LeftAnkle]";

    const auto& kp2d = inputs.camera_a_pose.keypoints[kLA];
    const auto& kp3d = inputs.camera_a_pose_3d.model_depth[kLA];

    r.x = kp2d.pixel.x; r.y = kp2d.pixel.y;
    r.z = kp3d.z_decoded ? kp3d.refined_z : 0.0f;
    r.xyz_valid        = kp2d.present;
    r.confidence       = kp2d.confidence;
    r.model_z_decoded  = kp3d.z_decoded;
    r.model_raw_z      = kp3d.raw_z_bin;
    r.measurement_driven = inputs.camera_a_pose.valid && kp2d.present;

    if (!inputs.camera_a_pose.valid) {
        r.z_state = ZState::Absent; r.reason_if_no_value = "camera_a_pose.valid=false";
        r.depth_source = "none";
    } else if (!kp3d.z_decoded) {
        r.z_state = ZState::Absent; r.reason_if_no_value = "camera_a_pose_3d.valid=false";
        r.depth_source = "none";
        r.transform_note = "solver_receives_2D_only;stereo_triangulation_path";
    } else if (!std::isfinite(kp3d.refined_z)) {
        r.z_state = ZState::Invalid; r.depth_source = "model_depth_hint_body_relative";
    } else if (kp3d.refined_z == 0.0f) {
        r.z_state = ZState::Zeroed; r.depth_source = "model_depth_hint_body_relative";
    } else {
        r.z_state = ZState::Present; r.depth_source = "model_depth_hint_body_relative";
        r.transform_note = "z_NOT_metric_world;applied_as_monocular_depth_offset_via_ApplyModelDepthToMonocularSeeds";
    }
    return r;
}

// ── Stage 4: UnifiedBodyState / BodyJointRole ─────────────────────────────────

inline DepthTraceRecord TraceUnifiedBodyState(
    std::uint64_t frame_id,
    const UnifiedBodyState& body_state)
{
    DepthTraceRecord r;
    r.frame_id         = frame_id;
    r.stage            = DepthTraceStage::UnifiedBodyState;
    r.predecessor      = DepthTraceStage::BodySolverInput;
    r.coordinate_frame = "world_m";
    r.source_note      = "UnifiedBodyState::roles[LeftAnkle].position";

    const auto& joint = body_state.roles[BodyJointRoleIndex(BodyJointRole::LeftAnkle)];
    r.x = joint.position.x; r.y = joint.position.y; r.z = joint.position.z;
    r.xyz_valid          = joint.valid;
    r.confidence         = joint.confidence;
    r.triangulated       = joint.triangulated;
    r.depth_inferred     = joint.depth_inferred;
    r.reprojection_err_px = joint.reprojection_error_px;
    r.measurement_driven = joint.measured;

    switch (joint.depth_source) {
    case DepthSource::TriangulatedStereo:  r.depth_source = "triangulated_stereo";  break;
    case DepthSource::InferredMonocular:   r.depth_source = "inferred_monocular";   break;
    default:                               r.depth_source = "none";                 break;
    }

    if (!joint.valid) {
        r.z_state = ZState::Absent; r.reason_if_no_value = "joint.valid=false";
    } else if (!std::isfinite(joint.position.z)) {
        r.z_state = ZState::Invalid;
    } else if (joint.position.z == 0.0f) {
        r.z_state = ZState::Zeroed;
        r.reason_if_no_value = "z==0;ambiguous:genuine_floor_contact_or_flattened";
    } else if (joint.depth_inferred) {
        r.z_state = ZState::Inferred; r.transform_note = "monocular_inferred";
    } else if (joint.triangulated) {
        r.z_state = ZState::Present; r.transform_note = "stereo_triangulated;world_m";
    } else {
        r.z_state = ZState::Inferred; r.transform_note = "predicted_or_temporal_hold";
    }
    return r;
}

// ── Stage 5: tracker synthesis ────────────────────────────────────────────────

inline DepthTraceRecord TraceTrackerSynthesis(
    std::uint64_t frame_id,
    const TrackerPoseArray& trackers)
{
    DepthTraceRecord r;
    r.frame_id         = frame_id;
    r.stage            = DepthTraceStage::TrackerSynthesis;
    r.predecessor      = DepthTraceStage::UnifiedBodyState;
    r.coordinate_frame = "world_m";
    r.source_note      = "TrackerPose[LeftFoot].pose.position (pre-OSC-transform)";

    const auto& t = trackers[TrackerRoleIndex(TrackerRole::LeftFoot)];
    r.x = t.pose.position.x; r.y = t.pose.position.y; r.z = t.pose.position.z;
    r.xyz_valid          = t.valid;
    r.confidence         = t.confidence;
    r.measurement_driven = t.valid && t.confidence > 0.0f;

    if (!t.valid) {
        r.z_state = ZState::Absent; r.reason_if_no_value = "tracker.valid=false";
    } else if (!std::isfinite(t.pose.position.z)) {
        r.z_state = ZState::Invalid;
    } else if (t.pose.position.z == 0.0f) {
        r.z_state = ZState::Zeroed; r.reason_if_no_value = "z==0;check_UnifiedBodyState_upstream";
    } else {
        r.z_state = ZState::Present; r.transform_note = "world_m;pre_osc_tracker_space_transform";
    }
    return r;
}

// ── Stage 6: OSC candidate / final send ──────────────────────────────────────

inline DepthTraceRecord TraceOscCandidate(
    std::uint64_t frame_id,
    const TrackerPoseArray& trackers,
    const OscSendReport& report,
    const OscConfig& config)
{
    DepthTraceRecord r;
    r.frame_id         = frame_id;
    r.stage            = DepthTraceStage::OscCandidate;
    r.predecessor      = DepthTraceStage::TrackerSynthesis;
    r.coordinate_frame = "tracker_space";
    r.source_note      = "TransformTrackerPoseToTrackerSpace(LeftFoot)";

    const std::size_t li  = TrackerRoleIndex(TrackerRole::LeftFoot);
    const auto& t         = trackers[li];
    const auto& rpt       = report.roles[li];
    const Pose3f vr       = TransformTrackerPoseToTrackerSpace(t, config);

    r.x = vr.position.x; r.y = vr.position.y; r.z = vr.position.z;
    r.xyz_valid          = t.valid;
    r.confidence         = t.confidence;
    r.osc_sent           = rpt.sent;
    r.osc_rejected       = !rpt.sent && rpt.configured;
    r.osc_reject_reason  = rpt.reason;
    r.tracker_index      = rpt.tracker_index;
    r.osc_address        = "/tracking/trackers/" + std::to_string(rpt.tracker_index) + "/position";
    r.measurement_driven = t.valid;

    char scale_buf[32]; std::snprintf(scale_buf,sizeof(scale_buf),"%.4g",(double)config.tracker_space_scale);
    const std::string scale_note = std::string("tracker_space_scale=") + scale_buf;

    if (!t.valid) {
        r.z_state = ZState::Absent; r.reason_if_no_value = "tracker.valid=false";
    } else if (!std::isfinite(vr.position.z)) {
        r.z_state = ZState::Invalid;
    } else if (vr.position.z == 0.0f) {
        r.z_state = ZState::Zeroed; r.reason_if_no_value = "z==0_after_tracker_space_transform";
    } else {
        r.z_state        = ZState::Transformed;
        r.transform_note = "rotation+scale_world_to_tracker_space;" + scale_note;
    }
    return r;
}

// ── Convenience collector ─────────────────────────────────────────────────────

struct DepthProvenanceInputs {
    std::uint64_t           frame_id                  = 0;
    ModelDepthKeypoint      model_depth_left_ankle{};
    Keypoint2D              decoded_kp_left_ankle{};
    const BodySolveInputs*  solver_inputs             = nullptr;
    const UnifiedBodyState* body_state                = nullptr;
    const TrackerPoseArray* trackers                  = nullptr;
    const OscSendReport*    osc_report                = nullptr;
    const OscConfig*        osc_config                = nullptr;
};

inline DepthTraceReport CollectDepthTrace(const DepthProvenanceInputs& in) {
    std::vector<DepthTraceRecord> records;
    records.reserve(6);
    records.push_back(TraceModelOutput(in.frame_id, in.model_depth_left_ankle, in.decoded_kp_left_ankle.confidence));
    records.push_back(TraceDecodedKeypoint(in.frame_id, in.decoded_kp_left_ankle, in.model_depth_left_ankle));
    if (in.solver_inputs)                              records.push_back(TraceBodySolverInput(in.frame_id, *in.solver_inputs));
    if (in.body_state)                                 records.push_back(TraceUnifiedBodyState(in.frame_id, *in.body_state));
    if (in.trackers)                                   records.push_back(TraceTrackerSynthesis(in.frame_id, *in.trackers));
    if (in.trackers && in.osc_report && in.osc_config) records.push_back(TraceOscCandidate(in.frame_id, *in.trackers, *in.osc_report, *in.osc_config));
    return BuildReport(in.frame_id, std::move(records));
}

} // namespace bt
