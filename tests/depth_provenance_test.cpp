// depth_provenance_test.cpp
// Falsification harness for the depth-provenance invariant.
// Standalone: no OpenCV, no ONNX runtime needed.
//
// Tests:
//   1. Decode not dead-code  — synthetic z tensor reaches DecodedPose3D
//   2. Stage continuity      — all 6 stage records present
//   3. z preservation        — different source-z bins → different trace z values
//   4. No fake world mix     — decoded_keypoint coordinate_frame ≠ "world_m"
//   5. Rejected candidates   — downstream lossy stages appear in rejected_candidates
//   6. OpenVR safe           — Poll() does not crash; enabled-but-no-runtime returns gracefully

#include "debug/depth_trace.h"
#include "debug/depth_provenance.h"
#include "inference/rtmpose_decode.h"
#include "inference/rtmpose_model_contract.h"
#include "core/types.h"
#include "tracking/body_state.h"
#include "tracking/body_solver.h"
#include "tracking/tracker_synthesis.h"
#include "io/osc_sender.h"
#include "io/steamvr_provider.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace bt { namespace test {

static void Fail(const char* name, const char* reason) {
    std::fprintf(stderr, "FAIL [%s]: %s\n", name, reason);
    std::abort();
}
static void Pass(const char* name) {
    std::fprintf(stdout, "PASS [%s]\n", name);
}

// Build synthetic 3-tensor RTMW3D model output.
// Places a single strong peak at joint-15 (LeftAnkle in wholebody-133) at the given bins.
static std::vector<NamedTensorF32> MakeSyntheticRtmw3dOutputs(
    std::size_t x_peak, std::size_t y_peak, std::size_t z_peak)
{
    const std::size_t nj    = static_cast<std::size_t>(kRtmw3dWholeBodyKeypointCount);
    const std::size_t xbins = static_cast<std::size_t>(kRtmPoseHalpe26SimccXBins);
    const std::size_t ybins = static_cast<std::size_t>(kRtmPoseHalpe26SimccYBins);
    const std::size_t zbins = static_cast<std::size_t>(kRtmw3dSimccZBins);

    NamedTensorF32 tx, ty, tz;
    tx.name = "simcc_x"; tx.tensor.shape = {1,static_cast<int64_t>(nj),static_cast<int64_t>(xbins)};
    ty.name = "simcc_y"; ty.tensor.shape = {1,static_cast<int64_t>(nj),static_cast<int64_t>(ybins)};
    tz.name = "simcc_z"; tz.tensor.shape = {1,static_cast<int64_t>(nj),static_cast<int64_t>(zbins)};
    tx.tensor.data.assign(nj*xbins, 0.0f);
    ty.tensor.data.assign(nj*ybins, 0.0f);
    tz.tensor.data.assign(nj*zbins, 0.0f);
    const std::size_t j = 15; // LeftAnkle in 133-index
    if (x_peak < xbins) tx.tensor.data[j*xbins + x_peak] = 10.0f;
    if (y_peak < ybins) ty.tensor.data[j*ybins + y_peak] = 10.0f;
    if (z_peak < zbins) tz.tensor.data[j*zbins + z_peak] = 10.0f;
    return {tx, ty, tz};
}

static ImagePreprocessMeta MakeMeta() {
    ImagePreprocessMeta m;
    m.source_image_width  = 1280; m.source_image_height = 720;
    m.source_region       = {0.f,0.f,1280.f,720.f};
    m.model_input_width   = static_cast<int>(kRtmPoseHalpe26InputWidth);
    m.model_input_height  = static_cast<int>(kRtmPoseHalpe26InputHeight);
    m.resize_scale        = static_cast<float>(kRtmPoseHalpe26InputWidth)/1280.f;
    m.pad_left = 0.f; m.pad_top = 0.f;
    return m;
}

// ── Test 1 ────────────────────────────────────────────────────────────────────

static void Test1_DecodeNotDeadCode() {
    const char* N = "decode_not_dead_code";
    auto outputs = MakeSyntheticRtmw3dOutputs(50, 60, 100);
    auto meta    = MakeMeta();

    auto r = DecodeRtmPoseOutputsWithDepth(outputs, meta);
    if (!r.ok()) Fail(N, ("decode failed: "+r.status().message).c_str());

    const auto& res = r.value();
    if (!res.pose2d.valid)  Fail(N, "pose2d.valid=false");
    if (!res.pose3d.valid)  Fail(N, "pose3d.valid=false — outputs[2] was NOT decoded (dead code)");

    constexpr std::size_t kLA = static_cast<std::size_t>(KeypointId::LeftAnkle);
    const auto& dep = res.pose3d.model_depth[kLA];
    if (!dep.z_decoded) Fail(N, "model_depth[LeftAnkle].z_decoded=false — z SimCC bins not consumed");
    if (dep.raw_z_bin < 98.f || dep.raw_z_bin > 102.f) {
        char buf[64]; std::snprintf(buf,sizeof(buf),"raw_z_bin=%.1f, expected ~100",dep.raw_z_bin);
        Fail(N, buf);
    }
    if (res.pose3d.coordinate_frame != "model_simcc_body_relative")
        Fail(N, "coordinate_frame not set to model_simcc_body_relative");
    Pass(N);
}

// ── Test 2 ────────────────────────────────────────────────────────────────────

static void Test2_StageContinuity() {
    const char* N = "stage_continuity";
    auto meta = MakeMeta();
    auto dec  = DecodeRtmPoseOutputsWithDepth(MakeSyntheticRtmw3dOutputs(50,60,100), meta);
    if (!dec.ok()) Fail(N, "decode failed");

    constexpr std::size_t kLA = static_cast<std::size_t>(KeypointId::LeftAnkle);

    DepthProvenanceInputs pi;
    pi.frame_id               = 42;
    pi.model_depth_left_ankle = dec.value().pose3d.model_depth[kLA];
    pi.decoded_kp_left_ankle  = dec.value().pose2d.keypoints[kLA];

    BodySolveInputs bsi;
    bsi.camera_a_pose    = dec.value().pose2d;
    bsi.camera_a_pose_3d = dec.value().pose3d;
    pi.solver_inputs = &bsi;

    UnifiedBodyState ubs;
    ubs.valid = true;
    auto& jnt = ubs.roles[BodyJointRoleIndex(BodyJointRole::LeftAnkle)];
    jnt.role = BodyJointRole::LeftAnkle; jnt.position = {0.1f,0.05f,0.8f};
    jnt.confidence=0.9f; jnt.valid=true; jnt.triangulated=true;
    jnt.depth_source=DepthSource::TriangulatedStereo; jnt.measured=true;
    pi.body_state = &ubs;

    TrackerPoseArray trackers{};
    for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
        trackers[i] = TrackerPose{kTrackerRoles[i], Pose3f{}, 0.0f, false};
    }
    trackers[TrackerRoleIndex(TrackerRole::LeftFoot)] =
        TrackerPose{TrackerRole::LeftFoot, Pose3f{Vec3f{0.1f,0.05f,0.8f},Quatf{0,0,0,1}},0.9f,true};
    pi.trackers = &trackers;

    OscSendReport rpt;
    const std::size_t li = TrackerRoleIndex(TrackerRole::LeftFoot);
    rpt.roles[li].role=TrackerRole::LeftFoot; rpt.roles[li].tracker_index=1;
    rpt.roles[li].configured=true; rpt.roles[li].valid=true;
    rpt.roles[li].sent=true; rpt.roles[li].reason="ok";
    OscConfig cfg; cfg.tracker_space_scale=1.f; cfg.tracker_space_rotation={0,0,0,1};
    pi.osc_report = &rpt; pi.osc_config = &cfg;

    const auto report = CollectDepthTrace(pi);
    if (!report.has_model_output)      Fail(N,"missing model_output");
    if (!report.has_decoded_keypoint)  Fail(N,"missing decoded_keypoint");
    if (!report.has_body_solver_input) Fail(N,"missing body_solver_input");
    if (!report.has_unified_body_state)Fail(N,"missing unified_body_state");
    if (!report.has_tracker_synthesis) Fail(N,"missing tracker_synthesis");
    if (!report.has_osc_candidate)     Fail(N,"missing osc_candidate");
    if (report.records.size() < 6)     Fail(N,"fewer than 6 records");
    Pass(N);
}

// ── Test 3 ────────────────────────────────────────────────────────────────────

static void Test3_ZPreservation() {
    const char* N = "z_preservation";
    auto meta = MakeMeta();
    auto da = DecodeRtmPoseOutputsWithDepth(MakeSyntheticRtmw3dOutputs(50,60, 80), meta);
    auto db = DecodeRtmPoseOutputsWithDepth(MakeSyntheticRtmw3dOutputs(50,60,200), meta);
    if (!da.ok()||!db.ok()) Fail(N,"decode failed");

    constexpr std::size_t kLA = static_cast<std::size_t>(KeypointId::LeftAnkle);
    const auto& dpa = da.value().pose3d.model_depth[kLA];
    const auto& dpb = db.value().pose3d.model_depth[kLA];
    if (!dpa.z_decoded||!dpb.z_decoded) Fail(N,"z not decoded in one or both frames");

    const float delta = std::abs(dpb.refined_z - dpa.refined_z);
    if (delta < 50.f) {
        char buf[128];
        std::snprintf(buf,sizeof(buf),"z barely changed: delta=%.2f (refined_z_a=%.2f, refined_z_b=%.2f); expected >=50",
            delta, dpa.refined_z, dpb.refined_z);
        Fail(N, buf);
    }
    // Stage-1 trace records must reflect the change
    const auto ra = TraceModelOutput(1, dpa, 0.9f);
    const auto rb = TraceModelOutput(2, dpb, 0.9f);
    if (ra.z_state!=ZState::Present||rb.z_state!=ZState::Present)
        Fail(N,"z_state not Present in model_output for one frame");
    if (std::abs(rb.z - ra.z) < 50.f)
        Fail(N,"trace z values do not change across frames — z is being flattened");
    Pass(N);
}

// ── Test 4 ────────────────────────────────────────────────────────────────────

static void Test4_NoFakeWorldMix() {
    const char* N = "no_fake_world_mix";
    auto meta = MakeMeta();
    auto dec  = DecodeRtmPoseOutputsWithDepth(MakeSyntheticRtmw3dOutputs(50,60,100), meta);
    if (!dec.ok()) Fail(N,"decode failed");
    constexpr std::size_t kLA = static_cast<std::size_t>(KeypointId::LeftAnkle);
    const auto rec = TraceDecodedKeypoint(1, dec.value().pose2d.keypoints[kLA], dec.value().pose3d.model_depth[kLA]);
    const std::string& cf = rec.coordinate_frame;
    if (cf.find("world_m") != std::string::npos || cf.find("metric") != std::string::npos)
        Fail(N, ("coordinate_frame incorrectly labelled as metric world: "+cf).c_str());
    if (rec.z_state == ZState::Present) {
        const std::string& tn = rec.transform_note;
        if (tn.find("NOT_metric") == std::string::npos && tn.find("not_metric") == std::string::npos)
            Fail(N, ("transform_note must warn about non-metric z, got: '"+tn+"'").c_str());
    }
    Pass(N);
}

// ── Test 5 ────────────────────────────────────────────────────────────────────

static void Test5_RejectedCandidateProvenance() {
    const char* N = "rejected_candidate_provenance";
    std::vector<DepthTraceRecord> recs;
    auto mk = [](std::uint64_t fid, DepthTraceStage st, ZState zs, const char* reason="") {
        DepthTraceRecord r; r.frame_id=fid; r.stage=st; r.z_state=zs;
        r.reason_if_no_value=reason; r.xyz_valid=true; return r;
    };
    recs.push_back(mk(99, DepthTraceStage::ModelOutput,      ZState::Present));
    recs.push_back(mk(99, DepthTraceStage::DecodedKeypoint,  ZState::Absent,"pose3d_not_wired"));
    recs.push_back(mk(99, DepthTraceStage::BodySolverInput,  ZState::Absent));
    recs.push_back(mk(99, DepthTraceStage::UnifiedBodyState, ZState::Inferred));

    auto rep = BuildReport(99, std::move(recs));
    if (rep.first_lossy_stage != "decoded_keypoint")
        Fail(N,("wrong first_lossy_stage: "+rep.first_lossy_stage).c_str());

    bool has_bsi=false, has_ubs=false;
    for (const auto& rc : rep.rejected_candidates) {
        if (rc.stage=="body_solver_input")  has_bsi=true;
        if (rc.stage=="unified_body_state") has_ubs=true;
    }
    if (!has_bsi) Fail(N,"body_solver_input missing from rejected_candidates");
    if (!has_ubs) Fail(N,"unified_body_state missing from rejected_candidates");
    Pass(N);
}

// ── Test 6 ────────────────────────────────────────────────────────────────────

static void Test6_OpenVrSafe() {
    const char* N = "openvr_safe_init";

    // FakeSteamVrPoseProvider — always safe
    {
        FakeSteamVrPoseProvider fake(MakeUnavailableSnapshot("test_no_runtime"));
        auto s = fake.Poll();
        if (s.available) Fail(N,"fake unavailable snapshot reported available");
    }
    {
        SteamVrPoseSnapshot valid{};
        valid.available=true; valid.runtime_initialized=true; valid.status="ok";
        FakeSteamVrPoseProvider fake(valid);
        if (!fake.Poll().available) Fail(N,"fake valid snapshot not returned");
    }

    // Real SteamVrPoseProvider — must not crash in any environment.
    // Whether or not OpenVR runtime is present, Poll() must return without aborting.
    {
        SteamVrPoseProvider provider;
        auto s = provider.Poll(); (void)s;
    }
    // Multiple calls — backoff / retry must not dereference null vr_system_
    {
        SteamVrPoseProvider provider;
        for (int i=0;i<4;++i) { auto s=provider.Poll(); (void)s; }
    }
    Pass(N);
}

}} // namespace bt::test

int main() {
    using namespace bt::test;
    Test1_DecodeNotDeadCode();
    Test2_StageContinuity();
    Test3_ZPreservation();
    Test4_NoFakeWorldMix();
    Test5_RejectedCandidateProvenance();
    Test6_OpenVrSafe();
    std::fprintf(stdout,"\nAll depth_provenance tests PASSED.\n");
    return 0;
}
