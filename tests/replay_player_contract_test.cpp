#include "debug/replay_player.h"
#include "test_check.h"

#include <filesystem>
#include <fstream>
#include <string>

int main() {
    const auto path = std::filesystem::temp_directory_path() / "bodytracker_replay_player_contract_test.ndjson";
    {
        std::ofstream out(path);
        BT_CHECK(static_cast<bool>(out));
        const std::string manifest_json = R"json({
            "type": "replay_manifest",
            "format": "bodytracker.ndjson.replay",
            "schema_version": 2,
            "deterministic": true,
            "model_path": "models/rtmw-dw-x-l-cocktail14-384x288.onnx",
            "calibration_path": "calib/default.json",
            "config": {
                "path": "config/default.json",
                "json": "{\"tracking\":{\"enable_replay_recording\":true}}",
                "hash_fnv1a64": "abc123",
                "recorded": true
            }
        })json";
        for (const char ch : manifest_json) {
            if (ch != '\n' && ch != '\r') {
                out << ch;
            }
        }
        out << '\n';
        const std::string replay_json = R"json({
            "type": "replay_frame",
            "schema_version": 2,
            "frame_index": 0,
            "deterministic": true,
            "phase": "runtime_input",
            "timestamp_seconds": 42.25,
            "degradation_mode": "partial_frame",
            "last_error": "network camera JPEG payload stalled; keeping last finite frame available",
            "camera_a": {
                "opened": true,
                "running": true,
                "delivered": 7,
                "read_failures": 2,
                "width": 1280,
                "height": 720
            },
            "camera_b": {
                "opened": true,
                "running": true,
                "delivered": 6,
                "read_failures": 1,
                "width": 1280,
                "height": 720
            },
            "frame_ages_ms": {
                "camera_a": 33.5,
                "camera_b": 48.25
            },
            "frame_pairing": {
                "accepted_pairs": 5,
                "missing_a": 1,
                "missing_b": 0,
                "rejected_skew": 2,
                "last_skew_ms": 17.75
            },
            "timing": {
                "inference_ms": 9.5,
                "pipeline_ms": 2.25,
                "osc_ms": 0.75
            },
            "osc": {
                "enabled": true,
                "open": true,
                "last_send_ok": false,
                "status": "partial_sent",
                "last_error": "pelvis position: injected pelvis send failure",
                "target_address": "127.0.0.1",
                "target_port": 9000,
                "tracker_space_transform_valid": true,
                "tracker_space_source": "steamvr_controller_alignment_stale",
                "manual_tracker_space_fallback_valid": true,
                "manual_tracker_space_source": "manual",
                "sent_tracker_count": 4,
                "skipped_tracker_count": 1,
                "sent_message_count": 4,
                "roles": [
                    {
                        "role": "pelvis",
                        "tracker_index": 1,
                        "configured": true,
                        "valid": true,
                        "sent": false,
                        "reason": "transport_failed"
                    },
                    {
                        "role": "right_knee",
                        "tracker_index": 5,
                        "configured": true,
                        "valid": true,
                        "sent": true,
                        "reason": "sent"
                    }
                ]
            },
            "steamvr_alignment": {
                "recorded": true,
                "provider": {
                    "available": true,
                    "runtime_initialized": true,
                    "status": "degraded",
                    "reason": "controller_pose_age_high",
                    "left_controller_tracked": true,
                    "right_controller_tracked": false,
                    "controller_device_count": 1,
                    "last_pose_age_seconds": 0.42,
                    "controller_alignment_fresh": false
                },
                "transform": {
                    "valid": true,
                    "stale": true,
                    "state": "stale_controller_alignment",
                    "reason": "using_last_numeric_transform",
                    "stale_reason": "controller_pose_age_high",
                    "confidence": 0.55,
                    "residual_m": 0.03,
                    "scale_ratio": 1.02,
                    "scale_mismatch": 0.02
                }
            },
            "hmd": {
                "valid": true
            }
        })json";
        for (const char ch : replay_json) {
            if (ch != '\n' && ch != '\r') {
                out << ch;
            }
        }
        out << '\n';
    }

    bt::ReplayPlayer player;
    const auto recording = player.LoadRecording(path);
    BT_CHECK(recording.ok());
    BT_CHECK(recording.value().metadata.schema_version == 2);
    BT_CHECK(recording.value().metadata.format == "bodytracker.ndjson.replay");
    BT_CHECK(recording.value().metadata.deterministic);
    BT_CHECK(recording.value().metadata.config_path == "config/default.json");
    BT_CHECK(recording.value().metadata.config_json.find("enable_replay_recording") != std::string::npos);
    BT_CHECK(recording.value().metadata.config_hash == "abc123");
    BT_CHECK(recording.value().metadata.model_path.find("rtmw-dw-x-l") != std::string::npos);
    BT_CHECK(recording.value().metadata.frame_count == 1);

    const auto loaded = player.Load(path);
    BT_CHECK(loaded.ok());
    BT_CHECK(loaded.value().size() == 1);
    const auto& frame = loaded.value().front();

    BT_CHECK(frame.phase == "runtime_input");
    BT_CHECK_NEAR(frame.timestamp_seconds, 42.25, 1e-6);
    BT_CHECK(frame.degradation_mode == "partial_frame");
    BT_CHECK(frame.last_error.find("keeping last finite frame") != std::string::npos);

    BT_CHECK(frame.camera_a.opened);
    BT_CHECK(frame.camera_a.running);
    BT_CHECK(frame.camera_a.delivered_frames == 7);
    BT_CHECK(frame.camera_a.read_failures == 2);
    BT_CHECK(frame.camera_a.actual_width == 1280);
    BT_CHECK(frame.camera_a.actual_height == 720);
    BT_CHECK_NEAR(frame.camera_a_frame_age_ms, 33.5, 1e-6);
    BT_CHECK_NEAR(frame.camera_b_frame_age_ms, 48.25, 1e-6);

    BT_CHECK(frame.frame_pairing.accepted_pairs == 5);
    BT_CHECK(frame.frame_pairing.missing_a == 1);
    BT_CHECK(frame.frame_pairing.rejected_skew == 2);
    BT_CHECK_NEAR(frame.frame_pairing.last_skew_ms, 17.75, 1e-6);
    BT_CHECK_NEAR(frame.inference_ms, 9.5, 1e-6);
    BT_CHECK_NEAR(frame.pipeline_ms, 2.25, 1e-6);
    BT_CHECK_NEAR(frame.osc_ms, 0.75, 1e-6);

    BT_CHECK(frame.osc_enabled);
    BT_CHECK(frame.osc_open);
    BT_CHECK(!frame.osc_last_send_ok);
    BT_CHECK(frame.osc_status == "partial_sent");
    BT_CHECK(frame.osc_tracker_space_transform_valid);
    BT_CHECK(frame.osc_tracker_space_source == "steamvr_controller_alignment_stale");
    BT_CHECK(frame.osc_manual_tracker_space_fallback_valid);
    BT_CHECK(frame.osc_sent_tracker_count == 4);
    BT_CHECK(frame.osc_skipped_tracker_count == 1);
    BT_CHECK(frame.osc_role_reasons[bt::TrackerRoleIndex(bt::TrackerRole::Pelvis)] == "transport_failed");
    BT_CHECK(frame.osc_role_sent[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)]);

    BT_CHECK(frame.steamvr_alignment_recorded);
    BT_CHECK(frame.steamvr_alignment.provider_available);
    BT_CHECK(frame.steamvr_alignment.provider_status == "degraded");
    BT_CHECK(frame.steamvr_alignment.provider_reason == "controller_pose_age_high");
    BT_CHECK(!frame.steamvr_alignment.controller_alignment_fresh);
    BT_CHECK(frame.steamvr_alignment.stale);
    BT_CHECK(frame.steamvr_alignment.state == "stale_controller_alignment");
    BT_CHECK(frame.steamvr_alignment.stale_reason == "controller_pose_age_high");
    BT_CHECK_NEAR(frame.steamvr_alignment.confidence, 0.55, 1e-6);
    BT_CHECK(frame.hmd_valid);

    const auto seek = player.SeekAtOrAfter(recording.value(), 40.0);
    BT_CHECK(seek.ok());
    BT_CHECK_NEAR(seek.value().timestamp_seconds, 42.25, 1e-6);

    std::filesystem::remove(path);
    return 0;
}
