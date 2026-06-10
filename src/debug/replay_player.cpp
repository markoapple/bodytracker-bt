#include "debug/replay_player.h"

#include <fstream>
#include <utility>
#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
#else
#include "nlohmann_json.hpp"
#endif

namespace bt {

Result<std::vector<DebugSnapshot>> ReplayPlayer::Load(const std::filesystem::path& path) {
    auto recording = LoadRecording(path);
    if (!recording.ok()) {
        return recording.status();
    }
    return std::move(recording.value().frames);
}

Result<ReplayRecording> ReplayPlayer::LoadRecording(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return Status::Error(StatusCode::InvalidArgument, "Replay file not readable");
    }
    ReplayRecording recording;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        DebugSnapshot snap;
        try {
            const auto j = nlohmann::json::parse(line);
            const std::string record_type = j.value("type", "replay_frame");
            if (record_type == "replay_manifest") {
                recording.metadata.schema_version = j.value("schema_version", 1);
                recording.metadata.format = j.value("format", std::string());
                recording.metadata.deterministic = j.value("deterministic", false);
                recording.metadata.model_path = j.value("model_path", std::string());
                recording.metadata.calibration_path = j.value("calibration_path", std::string());
                const auto config = j.value("config", nlohmann::json::object());
                recording.metadata.config_path = config.value("path", std::string());
                recording.metadata.config_json = config.value("json", std::string());
                recording.metadata.config_hash = config.value("hash_fnv1a64", std::string());
                continue;
            }
            snap.phase = j.value("phase", "replay");
            snap.timestamp_seconds = j.value("timestamp_seconds", 0.0);
            snap.degradation_mode = j.value("degradation_mode", "");
            snap.last_error = j.value("last_error", "");

            const auto camera_a = j.value("camera_a", nlohmann::json::object());
            snap.camera_a.opened = camera_a.value("opened", false);
            snap.camera_a.running = camera_a.value("running", false);
            snap.camera_a.delivered_frames = camera_a.value("delivered", 0ull);
            snap.camera_a.read_failures = camera_a.value("read_failures", 0ull);
            snap.camera_a.actual_width = camera_a.value("width", 0);
            snap.camera_a.actual_height = camera_a.value("height", 0);

            const auto camera_b = j.value("camera_b", nlohmann::json::object());
            snap.camera_b.opened = camera_b.value("opened", false);
            snap.camera_b.running = camera_b.value("running", false);
            snap.camera_b.delivered_frames = camera_b.value("delivered", 0ull);
            snap.camera_b.read_failures = camera_b.value("read_failures", 0ull);
            snap.camera_b.actual_width = camera_b.value("width", 0);
            snap.camera_b.actual_height = camera_b.value("height", 0);

            const auto frame_ages = j.value("frame_ages_ms", nlohmann::json::object());
            snap.camera_a_frame_age_ms = frame_ages.value("camera_a", 0.0);
            snap.camera_b_frame_age_ms = frame_ages.value("camera_b", 0.0);

            const auto frame_pairing = j.value("frame_pairing", nlohmann::json::object());
            snap.frame_pairing.accepted_pairs = frame_pairing.value("accepted_pairs", 0ull);
            snap.frame_pairing.missing_a = frame_pairing.value("missing_a", 0ull);
            snap.frame_pairing.missing_b = frame_pairing.value("missing_b", 0ull);
            snap.frame_pairing.rejected_skew = frame_pairing.value("rejected_skew", 0ull);
            snap.frame_pairing.rejected_duplicate = frame_pairing.value("rejected_duplicate", 0ull);
            snap.frame_pairing.rejected_reused_a = frame_pairing.value("rejected_reused_a", 0ull);
            snap.frame_pairing.rejected_reused_b = frame_pairing.value("rejected_reused_b", 0ull);
            snap.frame_pairing.degraded_skew = frame_pairing.value("degraded_skew", 0ull);
            snap.frame_pairing.degraded_duplicate = frame_pairing.value("degraded_duplicate", 0ull);
            snap.frame_pairing.degraded_reused_a = frame_pairing.value("degraded_reused_a", 0ull);
            snap.frame_pairing.degraded_reused_b = frame_pairing.value("degraded_reused_b", 0ull);
            snap.frame_pairing.last_accepted_sequence_a = frame_pairing.value("last_accepted_sequence_a", 0ull);
            snap.frame_pairing.last_accepted_sequence_b = frame_pairing.value("last_accepted_sequence_b", 0ull);
            snap.frame_pairing.last_skew_ms = frame_pairing.value("last_skew_ms", 0.0);
            snap.frame_pair_degraded = frame_pairing.value("current_degraded", false);
            snap.frame_pair_reused_a = frame_pairing.value("current_reused_a", false);
            snap.frame_pair_reused_b = frame_pairing.value("current_reused_b", false);
            snap.frame_pair_duplicate = frame_pairing.value("current_duplicate", false);
            snap.frame_pair_skewed = frame_pairing.value("current_skewed", false);
            snap.frame_pair_reason = frame_pairing.value("current_reason", std::string());

            const auto timing = j.value("timing", nlohmann::json::object());
            snap.inference_ms = timing.value("inference_ms", 0.0);
            snap.pipeline_ms = timing.value("pipeline_ms", 0.0);
            snap.osc_ms = timing.value("osc_ms", 0.0);

            const auto osc = j.value("osc", nlohmann::json::object());
            snap.osc_enabled = osc.value("enabled", false);
            snap.osc_open = osc.value("open", false);
            snap.osc_last_send_ok = osc.value("last_send_ok", false);
            snap.osc_status = osc.value("status", snap.osc_enabled ? (snap.osc_open ? "idle" : "closed") : "disabled");
            snap.osc_last_error = osc.value("last_error", "");
            snap.osc_open_attempts.clear();
            const auto open_attempts = osc.value("open_attempts", nlohmann::json::array());
            for (const auto& attempt : open_attempts) {
                if (attempt.is_string()) {
                    snap.osc_open_attempts.push_back(attempt.get<std::string>());
                }
            }
            snap.osc_target_address = osc.value("target_address", "");
            snap.osc_target_port = osc.value("target_port", 0);
            snap.osc_tracker_space_transform_valid = osc.value("tracker_space_transform_valid", false);
            snap.osc_tracker_space_source = osc.value("tracker_space_source", std::string("manual"));
            snap.osc_manual_tracker_space_fallback_valid = osc.value("manual_tracker_space_fallback_valid", false);
            snap.osc_manual_tracker_space_source = osc.value("manual_tracker_space_source", std::string("manual"));
            snap.osc_sent_tracker_count = osc.value("sent_tracker_count", 0);
            snap.osc_skipped_tracker_count = osc.value("skipped_tracker_count", 0);
            snap.osc_sent_message_count = osc.value("sent_message_count", 0);
            const auto roles = osc.value("roles", nlohmann::json::array());
            for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
                snap.osc_role_indices[i] = 0;
                snap.osc_role_configured[i] = false;
                snap.osc_role_valid[i] = false;
                snap.osc_role_sent[i] = false;
                snap.osc_role_degraded[i] = false;
                snap.osc_role_reasons[i] = "not_recorded";
                snap.osc_role_error_details[i].clear();
            }
            for (std::size_t entry_index = 0; entry_index < roles.size(); ++entry_index) {
                const auto& entry = roles[entry_index];
                if (!entry.is_object()) {
                    continue;
                }
                std::size_t role_index = entry_index < kTrackerRoles.size() ? entry_index : kTrackerPoseCount;
                const std::string role_name = entry.value("role", "");
                for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
                    if (role_name == ToString(kTrackerRoles[i])) {
                        role_index = i;
                        break;
                    }
                }
                if (role_index >= kTrackerPoseCount) {
                    continue;
                }
                snap.osc_role_indices[role_index] = entry.value("tracker_index", 0);
                snap.osc_role_configured[role_index] = entry.value("configured", false);
                snap.osc_role_valid[role_index] = entry.value("valid", false);
                snap.osc_role_sent[role_index] = entry.value("sent", false);
                snap.osc_role_degraded[role_index] = entry.value("degraded", false);
                snap.osc_role_reasons[role_index] = entry.value("reason", "");
                snap.osc_role_error_details[role_index] = entry.value("error_detail", "");
            }

            const auto steamvr_alignment = j.value("steamvr_alignment", nlohmann::json::object());
            snap.steamvr_alignment_recorded = steamvr_alignment.value("recorded", false);
            if (snap.steamvr_alignment_recorded) {
                const auto provider = steamvr_alignment.value("provider", nlohmann::json::object());
                snap.steamvr_alignment.provider_available = provider.value("available", false);
                snap.steamvr_alignment.provider_runtime_initialized = provider.value("runtime_initialized", false);
                snap.steamvr_alignment.provider_status = provider.value("status", std::string("unavailable"));
                snap.steamvr_alignment.provider_reason = provider.value("reason", std::string("not_recorded"));
                snap.steamvr_alignment.left_controller_tracked = provider.value("left_controller_tracked", false);
                snap.steamvr_alignment.right_controller_tracked = provider.value("right_controller_tracked", false);
                snap.steamvr_alignment.controller_device_count = provider.value("controller_device_count", 0);
                snap.steamvr_alignment.last_pose_age_seconds = provider.value("last_pose_age_seconds", 0.0);
                snap.steamvr_alignment.controller_alignment_fresh = provider.value("controller_alignment_fresh", false);
                const auto transform = steamvr_alignment.value("transform", nlohmann::json::object());
                snap.steamvr_alignment.transform_valid = transform.value("valid", false);
                snap.steamvr_alignment.stale = transform.value("stale", false);
                snap.steamvr_alignment.state = transform.value("state", std::string("missing"));
                snap.steamvr_alignment.reason = transform.value("reason", std::string());
                snap.steamvr_alignment.stale_reason = transform.value("stale_reason", std::string());
                snap.steamvr_alignment.confidence = transform.value("confidence", 0.0f);
                snap.steamvr_alignment.residual_m = transform.value("residual_m", 0.0f);
                snap.steamvr_alignment.scale_ratio = transform.value("scale_ratio", 1.0f);
                snap.steamvr_alignment.scale_mismatch = transform.value("scale_mismatch", 0.0f);
            }

            const auto hmd = j.value("hmd", nlohmann::json::object());
            snap.hmd_valid = hmd.value("valid", false);
        } catch (const std::exception&) {
            snap.phase = line;
        }
        if (recording.frames.empty()) {
            recording.metadata.first_timestamp_seconds = snap.timestamp_seconds;
        }
        recording.metadata.last_timestamp_seconds = snap.timestamp_seconds;
        recording.frames.push_back(snap);
    }
    recording.metadata.frame_count = recording.frames.size();
    if (recording.metadata.format.empty()) {
        recording.metadata.format = "legacy_bodytracker_ndjson";
    }
    if (recording.metadata.schema_version <= 0) {
        recording.metadata.schema_version = 1;
    }
    return recording;
}

Result<DebugSnapshot> ReplayPlayer::SeekAtOrAfter(const ReplayRecording& recording, double timestamp_seconds) const {
    if (recording.frames.empty()) {
        return Status::Error(StatusCode::FailedPrecondition, "Replay recording contains no frames");
    }
    for (const auto& frame : recording.frames) {
        if (frame.timestamp_seconds >= timestamp_seconds) {
            return frame;
        }
    }
    return recording.frames.back();
}

} // namespace bt
