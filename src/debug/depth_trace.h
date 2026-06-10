#pragma once
// depth_trace.h
// Machine-readable JSONL depth-provenance tracing for the LeftAnkle z path.
// Each pipeline stage boundary emits one DepthTraceRecord.
// Output: one JSON object per line (JSONL).

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace bt {

// Six mandatory stage boundaries
enum class DepthTraceStage : std::uint8_t {
    ModelOutput       = 0,
    DecodedKeypoint   = 1,
    BodySolverInput   = 2,
    UnifiedBodyState  = 3,
    TrackerSynthesis  = 4,
    OscCandidate      = 5,
};
inline const char* ToString(DepthTraceStage s) {
    switch (s) {
    case DepthTraceStage::ModelOutput:      return "model_output";
    case DepthTraceStage::DecodedKeypoint:  return "decoded_keypoint";
    case DepthTraceStage::BodySolverInput:  return "body_solver_input";
    case DepthTraceStage::UnifiedBodyState: return "unified_body_state";
    case DepthTraceStage::TrackerSynthesis: return "tracker_synthesis";
    case DepthTraceStage::OscCandidate:     return "osc_candidate";
    default:                                return "unknown";
    }
}

// Mandatory z classification
enum class ZState : std::uint8_t {
    Absent      = 0,   // no z value exists at this stage
    Zeroed      = 1,   // z is literal 0 (init / flatten)
    Constant    = 2,   // z does not change across samples where source z changes
    Transformed = 3,   // z changed due to a named coordinate transform
    Inferred    = 4,   // z was synthesised from other constraints
    Present     = 5,   // z is a real carried measurement
    Invalid     = 6,   // NaN / Inf / out-of-contract
};
inline const char* ToString(ZState z) {
    switch (z) {
    case ZState::Absent:      return "absent";
    case ZState::Zeroed:      return "zeroed";
    case ZState::Constant:    return "constant";
    case ZState::Transformed: return "transformed";
    case ZState::Inferred:    return "inferred";
    case ZState::Present:     return "present";
    case ZState::Invalid:     return "invalid";
    default:                  return "unknown";
    }
}

struct DepthTraceRecord {
    std::uint64_t    frame_id        = 0;
    std::string      joint_name      = "left_ankle";
    DepthTraceStage  stage           = DepthTraceStage::ModelOutput;
    DepthTraceStage  predecessor     = DepthTraceStage::ModelOutput;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    bool  xyz_valid                  = false;
    std::string coordinate_frame;
    ZState      z_state              = ZState::Absent;
    std::string reason_if_no_value;
    std::string transform_note;
    std::string source_note;
    float confidence                 = 0.0f;
    bool  measurement_driven         = false;
    // Model-layer metadata
    float model_raw_z                = 0.0f;
    bool  model_z_decoded            = false;
    // Solver-layer metadata
    std::string depth_source;
    bool triangulated                = false;
    bool depth_inferred              = false;
    float reprojection_err_px        = 0.0f;
    // OSC-layer metadata
    bool  osc_sent                   = false;
    bool  osc_rejected               = false;
    std::string osc_reject_reason;
    std::string osc_address;
    int   tracker_index              = -1;
};

struct DepthTraceReport {
    std::uint64_t frame_id           = 0;
    std::vector<DepthTraceRecord> records;
    bool has_model_output            = false;
    bool has_decoded_keypoint        = false;
    bool has_body_solver_input       = false;
    bool has_unified_body_state      = false;
    bool has_tracker_synthesis       = false;
    bool has_osc_candidate           = false;
    std::string first_lossy_stage;
    std::string first_lossy_reason;
    std::string upstream_last_good_stage;
    std::string downstream_first_bad_stage;
    struct RejectedCandidate { std::string stage; std::string rejection_reason; };
    std::vector<RejectedCandidate> rejected_candidates;
};

// ── JSONL serialization ───────────────────────────────────────────────────────

inline std::string JE(const std::string& s) {
    std::string o; o.reserve(s.size()+4);
    for (char c : s) {
        if      (c == '"')  { o += "\\\""; }
        else if (c == '\\') { o += "\\\\"; }
        else if (c == '\n') { o += "\\n"; }
        else                { o += c; }
    }
    return o;
}
inline std::string JF(float v)        { char b[32]; std::snprintf(b,sizeof(b),"%.6g",(double)v); return b; }
inline std::string JU(std::uint64_t v){ char b[24]; std::snprintf(b,sizeof(b),"%llu",(unsigned long long)v); return b; }
inline std::string JB(bool v)         { return v?"true":"false"; }
inline std::string JI(int v)          { char b[16]; std::snprintf(b,sizeof(b),"%d",v); return b; }

inline std::string RecordToJsonLine(const DepthTraceRecord& r) {
    std::string j; j.reserve(640);
    j+="{\"frame_id\":";      j+=JU(r.frame_id);
    j+=",\"joint\":\"";       j+=JE(r.joint_name); j+="\"";
    j+=",\"stage\":\"";       j+=ToString(r.stage); j+="\"";
    j+=",\"predecessor\":\""; j+=ToString(r.predecessor); j+="\"";
    j+=",\"xyz_valid\":";     j+=JB(r.xyz_valid);
    j+=",\"x\":";             j+=JF(r.x);
    j+=",\"y\":";             j+=JF(r.y);
    j+=",\"z\":";             j+=JF(r.z);
    j+=",\"coordinate_frame\":\""; j+=JE(r.coordinate_frame); j+="\"";
    j+=",\"z_state\":\"";     j+=ToString(r.z_state); j+="\"";
    j+=",\"reason_if_no_value\":\""; j+=JE(r.reason_if_no_value); j+="\"";
    j+=",\"transform_note\":\"";     j+=JE(r.transform_note);     j+="\"";
    j+=",\"source_note\":\"";        j+=JE(r.source_note);        j+="\"";
    j+=",\"confidence\":";    j+=JF(r.confidence);
    j+=",\"measurement_driven\":"; j+=JB(r.measurement_driven);
    j+=",\"model_raw_z\":";   j+=JF(r.model_raw_z);
    j+=",\"model_z_decoded\":"; j+=JB(r.model_z_decoded);
    j+=",\"depth_source\":\""; j+=JE(r.depth_source); j+="\"";
    j+=",\"triangulated\":";  j+=JB(r.triangulated);
    j+=",\"depth_inferred\":"; j+=JB(r.depth_inferred);
    j+=",\"reprojection_err_px\":"; j+=JF(r.reprojection_err_px);
    j+=",\"osc_sent\":";      j+=JB(r.osc_sent);
    j+=",\"osc_rejected\":";  j+=JB(r.osc_rejected);
    j+=",\"osc_reject_reason\":\""; j+=JE(r.osc_reject_reason); j+="\"";
    j+=",\"osc_address\":\""; j+=JE(r.osc_address); j+="\"";
    j+=",\"tracker_index\":"; j+=JI(r.tracker_index);
    j+="}";
    return j;
}

inline std::string ReportToJsonLine(const DepthTraceReport& rep) {
    std::string j; j.reserve(1024);
    j+="{\"type\":\"depth_trace_report\"";
    j+=",\"frame_id\":";          j+=JU(rep.frame_id);
    j+=",\"has_model_output\":";      j+=JB(rep.has_model_output);
    j+=",\"has_decoded_keypoint\":";  j+=JB(rep.has_decoded_keypoint);
    j+=",\"has_body_solver_input\":"; j+=JB(rep.has_body_solver_input);
    j+=",\"has_unified_body_state\":";j+=JB(rep.has_unified_body_state);
    j+=",\"has_tracker_synthesis\":"; j+=JB(rep.has_tracker_synthesis);
    j+=",\"has_osc_candidate\":";     j+=JB(rep.has_osc_candidate);
    j+=",\"first_lossy_stage\":\"";   j+=JE(rep.first_lossy_stage);      j+="\"";
    j+=",\"first_lossy_reason\":\"";  j+=JE(rep.first_lossy_reason);     j+="\"";
    j+=",\"upstream_last_good_stage\":\""; j+=JE(rep.upstream_last_good_stage); j+="\"";
    j+=",\"downstream_first_bad_stage\":\""; j+=JE(rep.downstream_first_bad_stage); j+="\"";
    j+=",\"rejected_candidates\":[";
    for (std::size_t i=0;i<rep.rejected_candidates.size();++i){
        if(i) j+=",";
        j+="{\"stage\":\""; j+=JE(rep.rejected_candidates[i].stage); j+="\"";
        j+=",\"rejection_reason\":\""; j+=JE(rep.rejected_candidates[i].rejection_reason); j+="\"}";
    }
    j+="]}";
    return j;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

inline ZState ClassifyZ(float z, bool decoded) {
    if (!decoded)           return ZState::Absent;
    if (!std::isfinite(z))  return ZState::Invalid;
    if (z == 0.0f)          return ZState::Zeroed;
    return ZState::Present;
}

inline DepthTraceReport BuildReport(std::uint64_t frame_id, std::vector<DepthTraceRecord> records) {
    static constexpr std::array<DepthTraceStage,6> kOrder = {
        DepthTraceStage::ModelOutput, DepthTraceStage::DecodedKeypoint,
        DepthTraceStage::BodySolverInput, DepthTraceStage::UnifiedBodyState,
        DepthTraceStage::TrackerSynthesis, DepthTraceStage::OscCandidate };

    DepthTraceReport rep;
    rep.frame_id = frame_id;
    rep.records  = std::move(records);

    for (const auto& r : rep.records) {
        switch(r.stage){
        case DepthTraceStage::ModelOutput:      rep.has_model_output=true;        break;
        case DepthTraceStage::DecodedKeypoint:  rep.has_decoded_keypoint=true;    break;
        case DepthTraceStage::BodySolverInput:  rep.has_body_solver_input=true;   break;
        case DepthTraceStage::UnifiedBodyState: rep.has_unified_body_state=true;  break;
        case DepthTraceStage::TrackerSynthesis: rep.has_tracker_synthesis=true;   break;
        case DepthTraceStage::OscCandidate:     rep.has_osc_candidate=true;       break;
        }
    }

    std::string last_good;
    bool found = false;
    for (auto stage : kOrder) {
        const DepthTraceRecord* rec = nullptr;
        for (const auto& r : rep.records) if (r.stage==stage) { rec=&r; break; }
        if (!rec) {
            if (!found) {
                rep.first_lossy_stage = ToString(stage);
                rep.first_lossy_reason = "stage_record_absent";
                rep.upstream_last_good_stage = last_good;
                rep.downstream_first_bad_stage = ToString(stage);
                found = true;
            }
            continue;
        }
        const bool z_ok = rec->z_state==ZState::Present || rec->z_state==ZState::Transformed;
        if (!z_ok && !found) {
            rep.first_lossy_stage  = ToString(stage);
            rep.first_lossy_reason = rec->reason_if_no_value.empty()
                ? ("z_state="+std::string(ToString(rec->z_state))) : rec->reason_if_no_value;
            rep.upstream_last_good_stage = last_good;
            rep.downstream_first_bad_stage = ToString(stage);
            found = true;
        }
        if (z_ok) last_good = ToString(stage);
    }
    // Rejected candidates: lossy-z stages that are NOT the first-lossy one
    for (auto stage : kOrder) {
        const DepthTraceRecord* rec = nullptr;
        for (const auto& r : rep.records) if (r.stage==stage) { rec=&r; break; }
        if (!rec) continue;
        const bool z_ok = rec->z_state==ZState::Present || rec->z_state==ZState::Transformed;
        if (!z_ok && ToString(stage)!=rep.first_lossy_stage && found) {
            rep.rejected_candidates.push_back({
                ToString(stage),
                "not_first_lossy;upstream_already_lost_z;z_state="+std::string(ToString(rec->z_state))
            });
        }
    }
    return rep;
}

} // namespace bt
