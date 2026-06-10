#include "core/config.h"
#include "core/types.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
#else
#include "nlohmann_json.hpp"
#endif
#include <sstream>
#include <string>
#include <system_error>
#include <utility>

namespace bt {

bool ConfigValidationReport::HasOutcome(ConfigValidationOutcome outcome) const noexcept {
    return std::any_of(issues.begin(), issues.end(), [outcome](const ConfigValidationIssue& issue) {
        return issue.outcome == outcome;
    });
}

bool ConfigValidationReport::HasInvalid() const noexcept {
    return HasOutcome(ConfigValidationOutcome::Invalid);
}

const char* ToString(ConfigValidationOutcome outcome) noexcept {
    switch (outcome) {
    case ConfigValidationOutcome::Invalid: return "invalid";
    case ConfigValidationOutcome::Degraded: return "degraded";
    case ConfigValidationOutcome::MissingButDefaultable: return "missing-but-defaultable";
    case ConfigValidationOutcome::Warning: return "warning";
    }
    return "warning";
}

namespace {

void AddConfigIssue(
    ConfigValidationReport& report,
    ConfigValidationOutcome outcome,
    std::string path,
    std::string message) {

    report.issues.push_back(ConfigValidationIssue{outcome, std::move(path), std::move(message)});
}

bool ManualTrackerSpaceFallbackFinite(const OscConfig& cfg) {
    return cfg.manual_tracker_space_transform_valid &&
        TrackerSpaceTransformFinite(
            cfg.manual_tracker_space_position_offset,
            cfg.manual_tracker_space_rotation,
            cfg.manual_tracker_space_scale,
            cfg.manual_tracker_space_role_offsets);
}

Status WriteTextFileAtomically(const std::filesystem::path& path, const std::string& contents, const char* label) {
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code mkdir_ec;
        std::filesystem::create_directories(parent, mkdir_ec);
        if (mkdir_ec) {
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not create parent directory for ") + label + ": " + mkdir_ec.message());
        }
    }

    std::filesystem::path temp_path = path;
    temp_path += ".tmp";
    std::filesystem::path backup_path = path;
    backup_path += ".bak";

    auto remove_quietly = [](const std::filesystem::path& p) {
        std::error_code ec;
        std::filesystem::remove(p, ec);
    };

    {
        std::ofstream out(temp_path, std::ios::trunc);
        if (!out) {
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not write ") + label + " temp file: " + temp_path.string());
        }
        out << contents;
        if (!out) {
            remove_quietly(temp_path);
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not finish writing ") + label + " temp file: " + temp_path.string());
        }
    }

    std::error_code ec;
    const bool stale_backup_exists = std::filesystem::exists(backup_path, ec);
    if (ec) {
        remove_quietly(temp_path);
        return Status::Error(StatusCode::InvalidArgument, std::string("Could not inspect stale backup for ") + label + ": " + ec.message());
    }
    if (stale_backup_exists) {
        std::filesystem::remove(backup_path, ec);
        if (ec) {
            remove_quietly(temp_path);
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not clear stale backup for ") + label + ": " + ec.message());
        }
    }

    ec.clear();
    const bool target_exists = std::filesystem::exists(path, ec);
    if (ec) {
        remove_quietly(temp_path);
        return Status::Error(StatusCode::InvalidArgument, std::string("Could not inspect existing ") + label + " before save: " + ec.message());
    }

    bool backup_created = false;
    if (target_exists) {
        std::filesystem::rename(path, backup_path, ec);
        if (ec) {
            remove_quietly(temp_path);
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not move existing ") + label + " to backup before save: " + ec.message());
        }
        backup_created = true;
    }

    auto rollback_after_replace_failure = [&](const std::error_code& replace_ec) {
        std::error_code rollback_ec;
        std::filesystem::remove(path, rollback_ec);
        if (rollback_ec) {
            return Status::Error(StatusCode::InvalidArgument,
                std::string("Could not replace ") + label + ": " + replace_ec.message() +
                "; rollback cleanup failed: " + rollback_ec.message());
        }
        if (backup_created) {
            rollback_ec.clear();
            std::filesystem::rename(backup_path, path, rollback_ec);
            if (rollback_ec) {
                return Status::Error(StatusCode::InvalidArgument,
                    std::string("Could not replace ") + label + ": " + replace_ec.message() +
                    "; rollback also failed: " + rollback_ec.message());
            }
        }
        return Status::Error(StatusCode::InvalidArgument, std::string("Could not replace ") + label + ": " + replace_ec.message());
    };

    ec.clear();
    std::filesystem::rename(temp_path, path, ec);
    if (ec) {
        const Status rollback_status = rollback_after_replace_failure(ec);
        remove_quietly(temp_path);
        return rollback_status;
    }

    if (backup_created) {
        remove_quietly(backup_path);
    }
    return Status::OK();
}

constexpr const char* kBuiltinConfigSchemaJson = R"BTSCHEMA({
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "bodytracker-config.schema.json",
  "title": "bodytracker config",
  "type": "object",
  "additionalProperties": true,
  "properties": {
    "$schema": {
      "type": "string"
    },
    "app": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "log_file": {
          "type": "string"
        },
        "recording_dir": {
          "type": "string"
        }
      }
    },
    "debug": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "replay_log_path": {
          "type": "string"
        }
      }
    },
    "inference": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "model_path": {
          "type": "string"
        },
        "device": {
          "enum": [
            "cpu",
            "directml",
            "directml_strict"
          ]
        }
      }
    },
    "calibration_path": {
      "type": "string"
    },
    "hmd": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "mode": {
          "enum": [
            "null",
            "json_file",
            "steamvr"
          ]
        },
        "pose_json_path": {
          "type": "string"
        }
      }
    },
    "tracking": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "mode": {
          "enum": [
            "stereo",
            "monocular"
          ]
        },
        "model_path": {
          "type": "string"
        },
        "depth_postprocess_enabled": {
          "type": "boolean"
        },
        "depth_postprocess_model_path": {
          "type": "string"
        },
        "depth_postprocess_interval_frames": {
          "type": "integer",
          "minimum": 1
        },
        "depth_postprocess_allow_cpu_fallback": {
          "type": "boolean"
        },
        "calibration_path": {
          "type": "string"
        },
        "latest_frame_skew_tolerance_ms": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "max_frame_skew_ms": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "stale_frame_timeout_ms": {
          "type": "number",
          "exclusiveMinimum": 0,
          "description": "Freshness/confidence input. Stale finite frames are degraded evidence, not invalid config."
        },
        "min_triangulated_seed_count": {
          "type": "integer",
          "minimum": 1,
          "maximum": 26
        },
        "max_mean_reprojection_error_px": {
          "type": "number",
          "exclusiveMinimum": 0,
          "description": "Default must match src/tracking/tracking_constants.h::kReprojectionErrorMaxPx unless score normalization is updated."
        },
        "stereo_monocular_fallback_enabled": {
          "type": "boolean"
        },
        "use_legacy_solver": {
          "type": "boolean"
        },
        "enable_replay_recording": {
          "type": "boolean"
        },
        "body_calibration": {
          "$ref": "#/$defs/body_calibration"
        },
        "monocular": {
          "$ref": "#/$defs/monocular"
        },
        "motion_consistency": {
          "$ref": "#/$defs/motion_consistency"
        },
        "tracker_ekf": {
          "$ref": "#/$defs/tracker_ekf"
        },
        "temporal_update": {
          "$ref": "#/$defs/temporal_update"
        },
        "stereo_epipolar": {
          "$ref": "#/$defs/stereo_epipolar"
        },
        "stereo_triangulation": {
          "$ref": "#/$defs/stereo_triangulation"
        },
        "stereo_uncertainty": {
          "$ref": "#/$defs/stereo_uncertainty"
        },
        "solver_observation_weighting": {
          "$ref": "#/$defs/solver_observation_weighting"
        },
        "stereo_identity": {
          "$ref": "#/$defs/stereo_identity"
        }
      }
    },
    "osc": {
      "$ref": "#/$defs/osc"
    },
    "camera_a": {
      "$ref": "#/$defs/camera"
    },
    "camera_b": {
      "$ref": "#/$defs/camera"
    }
  },
  "$defs": {
    "body_calibration": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "enabled": {
          "type": "boolean"
        },
        "auto_persist": {
          "type": "boolean"
        },
        "required_seconds": {
          "type": "number",
          "minimum": 0.5,
          "maximum": 15.0
        },
        "min_overall_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "max_segment_cv": {
          "type": "number",
          "minimum": 0.02,
          "maximum": 0.4
        }
      }
    },
    "monocular": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "image_width": {
          "type": "integer",
          "exclusiveMinimum": 0
        },
        "image_height": {
          "type": "integer",
          "exclusiveMinimum": 0
        },
        "horizontal_fov_deg": {
          "type": "number",
          "minimum": 30.0,
          "maximum": 130.0
        },
        "user_height_m": {
          "type": "number",
          "minimum": 0.8,
          "maximum": 2.5
        },
        "camera_height_m": {
          "type": "number",
          "minimum": 0.1,
          "maximum": 3.5
        },
        "default_depth_m": {
          "type": "number",
          "minimum": 0.3,
          "maximum": 8.0
        },
        "depth_confidence_scale": {
          "type": "number",
          "exclusiveMinimum": 0.0,
          "exclusiveMaximum": 1.0
        },
        "min_keypoint_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "min_seed_count": {
          "type": "integer",
          "minimum": 1,
          "maximum": 26
        },
        "floor_scale_assist_enabled": {
          "type": "boolean",
          "description": "If enabled without usable spacing, runtime falls back with a missing-but-defaultable validation issue."
        },
        "floor_geometry_calibration_enabled": {
          "type": "boolean"
        },
        "floor_geometry_type": {
          "type": "string"
        },
        "floor_depth_line_spacing_m": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 2.0
        },
        "floor_depth_line_spacing_px": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 2000.0
        },
        "floor_depth_reference_y_px": {
          "type": "number",
          "minimum": 0.0
        },
        "floor_depth_reference_m": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 8.0
        },
        "floor_depth_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "floor_second_axis_spacing_m": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 2.0
        },
        "floor_geometry_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "floor_projective_homography_enabled": {
          "type": "boolean"
        },
        "floor_from_image": {
          "type": "array",
          "minItems": 9,
          "maxItems": 9,
          "items": {
            "type": "number"
          }
        },
        "image_from_floor": {
          "type": "array",
          "minItems": 9,
          "maxItems": 9,
          "items": {
            "type": "number"
          }
        },
        "floor_projective_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "floor_distortion_correction_enabled": {
          "type": "boolean"
        },
        "floor_distortion_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "floor_radial_k1": {
          "type": "number"
        },
        "floor_radial_k2": {
          "type": "number"
        },
        "floor_tangential_p1": {
          "type": "number"
        },
        "floor_tangential_p2": {
          "type": "number"
        },
        "floor_camera_orientation_enabled": {
          "type": "boolean"
        },
        "floor_camera_pitch_rad": {
          "type": "number",
          "minimum": -1.4,
          "maximum": 1.4
        },
        "floor_camera_roll_rad": {
          "type": "number",
          "minimum": -1.4,
          "maximum": 1.4
        },
        "floor_camera_orientation_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        }
      }
    },
    "motion_consistency": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "enabled": {
          "type": "boolean"
        },
        "confirm_frames": {
          "type": "integer",
          "minimum": 1,
          "maximum": 8
        },
        "min_motion_m": {
          "type": "number",
          "minimum": 0.0
        },
        "stationary_deadzone_m": {
          "type": "number",
          "minimum": 0.0
        },
        "max_direction_deviation_deg": {
          "type": "number",
          "exclusiveMinimum": 0.0,
          "maximum": 180.0
        },
        "max_lateral_deviation_ratio": {
          "type": "number",
          "minimum": 0.0
        },
        "max_speed_change_ratio": {
          "type": "number",
          "minimum": 1.0
        },
        "reject_confidence_decay_per_second": {
          "type": "number",
          "minimum": 0.0
        },
        "planted_foot_max_drift_m": {
          "type": "number",
          "minimum": 0.0
        },
        "planted_foot_release_confirm_frames": {
          "type": "integer",
          "minimum": 1,
          "maximum": 8
        },
        "contact_root_correction_gain": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "contact_root_max_correction_m": {
          "type": "number",
          "minimum": 0.0
        },
        "contact_root_max_residual_m": {
          "type": "number",
          "minimum": 0.0
        },
        "contact_root_max_disagreement_m": {
          "type": "number",
          "minimum": 0.0
        },
        "contact_root_min_alignment": {
          "type": "number",
          "minimum": -1.0,
          "maximum": 1.0
        },
        "contact_root_min_support_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "root_max_speed_mps": {
          "type": "number",
          "minimum": 0.0
        },
        "foot_max_speed_mps": {
          "type": "number",
          "minimum": 0.0
        },
        "root_max_accel_mps2": {
          "type": "number",
          "minimum": 0.0
        },
        "foot_max_accel_mps2": {
          "type": "number",
          "minimum": 0.0
        },
        "confirm_scale_m": {
          "type": "number",
          "exclusiveMinimum": 0.0
        },
        "confirm_frames_max": {
          "type": "integer",
          "minimum": 1,
          "maximum": 20
        },
        "one_euro_enabled": {
          "type": "boolean"
        },
        "one_euro_min_cutoff_hz": {
          "type": "number",
          "exclusiveMinimum": 0.0
        },
        "one_euro_beta": {
          "type": "number",
          "minimum": 0.0
        },
        "one_euro_d_cutoff_hz": {
          "type": "number",
          "exclusiveMinimum": 0.0
        }
      }
    },
    "tracker_ekf": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "enabled": {
          "type": "boolean"
        },
        "process_noise_mps2": {
          "type": "number",
          "minimum": 0.0
        },
        "min_measurement_variance_m2": {
          "type": "number",
          "exclusiveMinimum": 0.0
        },
        "max_measurement_variance_m2": {
          "type": "number",
          "exclusiveMinimum": 0.0
        },
        "support_variance_scale": {
          "type": "number",
          "exclusiveMinimum": 0.0,
          "maximum": 1.0
        },
        "missing_velocity_decay": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "foot_orientation_gain": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "mahalanobis_gate_enabled": {
          "type": "boolean"
        },
        "mahalanobis_gate_chi2": {
          "type": "number",
          "minimum": 0.0
        },
        "outlier_variance_scale": {
          "type": "number",
          "minimum": 1.0
        }
      }
    },
)BTSCHEMA"
R"BTSCHEMA(
    "temporal_update": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "free_gain": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "supported_gain": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "foot_free_gain": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "foot_supported_gain": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        }
      }
    },
    "osc": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "enabled": {
          "type": "boolean"
        },
        "target_address": {
          "type": "string"
        },
        "target_port": {
          "type": "integer",
          "minimum": 1,
          "maximum": 65535
        },
        "send_rotations": {
          "type": "boolean"
        },
        "min_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0,
          "description": "Default matches src/tracking/tracking_constants.h::kVisibleConfidenceThreshold."
        },
        "pelvis_tracker_index": {
          "type": "integer",
          "minimum": 1,
          "maximum": 8
        },
        "left_foot_tracker_index": {
          "type": "integer",
          "minimum": 1,
          "maximum": 8
        },
        "right_foot_tracker_index": {
          "type": "integer",
          "minimum": 1,
          "maximum": 8
        },
        "chest_tracker_index": {
          "type": "integer",
          "minimum": 0,
          "maximum": 8
        },
        "left_elbow_tracker_index": {
          "type": "integer",
          "minimum": 0,
          "maximum": 8
        },
        "right_elbow_tracker_index": {
          "type": "integer",
          "minimum": 0,
          "maximum": 8
        },
        "left_knee_tracker_index": {
          "type": "integer",
          "minimum": 0,
          "maximum": 8
        },
        "right_knee_tracker_index": {
          "type": "integer",
          "minimum": 0,
          "maximum": 8
        },
        "tracker_space_transform_valid": {
          "type": "boolean",
          "description": "False with OSC enabled is degraded/recoverable. Load config; runtime blocks OSC until finite tracker-space exists."
        },
        "tracker_space_position_offset": {
          "type": "array",
          "minItems": 3,
          "maxItems": 3,
          "items": {
            "type": "number"
          }
        },
        "tracker_space_rotation": {
          "type": "array",
          "minItems": 4,
          "maxItems": 4,
          "items": {
            "type": "number"
          },
          "description": "Quaternion [x,y,z,w]. C++ load validation additionally rejects zero-length quaternions."
        },
        "tracker_space_scale": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "tracker_space_role_offsets": {
          "type": "array",
          "minItems": 5,
          "maxItems": 8,
          "items": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
              "type": "number"
            }
          }
        },
        "tracker_space_source": {
          "type": "string"
        },
        "manual_tracker_space_transform_valid": {
          "type": "boolean"
        },
        "manual_tracker_space_position_offset": {
          "type": "array",
          "minItems": 3,
          "maxItems": 3,
          "items": {
            "type": "number"
          }
        },
        "manual_tracker_space_rotation": {
          "type": "array",
          "minItems": 4,
          "maxItems": 4,
          "items": {
            "type": "number"
          },
          "description": "Quaternion [x,y,z,w]. C++ load validation additionally rejects zero-length quaternions."
        },
        "manual_tracker_space_scale": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "manual_tracker_space_role_offsets": {
          "type": "array",
          "minItems": 5,
          "maxItems": 8,
          "items": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
              "type": "number"
            }
          }
        },
        "manual_tracker_space_source": {
          "type": "string"
        },
        "steamvr_alignment_status": {
          "type": "string"
        },
        "steamvr_alignment_reason": {
          "type": "string"
        },
        "steamvr_alignment_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "steamvr_alignment_residual_m": {
          "type": "number"
        },
        "steamvr_floor_residual_m": {
          "type": "number"
        },
        "steamvr_yaw_offset_rad": {
          "type": "number"
        },
        "steamvr_scale_ratio": {
          "type": "number",
          "exclusiveMinimum": 0.0
        },
        "steamvr_alignment_body_signature": {
          "type": "string"
        },
        "steamvr_alignment_floor_signature": {
          "type": "string"
        }
      }
    },
    "steamvr_tracker_bridge": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "enabled": {
          "type": "boolean"
        },
        "target_address": {
          "type": "string"
        },
        "target_port": {
          "type": "integer",
          "minimum": 1,
          "maximum": 65535
        },
        "min_confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        },
        "send_chest": {
          "type": "boolean"
        },
        "send_elbows": {
          "type": "boolean"
        },
        "send_knees": {
          "type": "boolean"
        }
      }
    },
    "camera": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "source": {
          "enum": [
            "opencv",
            "network_mjpeg"
          ]
        },
        "device_index": {
          "type": "integer",
          "minimum": -1
        },
        "width": {
          "type": "integer",
          "exclusiveMinimum": 0
        },
        "height": {
          "type": "integer",
          "exclusiveMinimum": 0
        },
        "fps": {
          "type": "integer",
          "exclusiveMinimum": 0
        },
        "network_bind_address": {
          "type": "string"
        },
        "network_port": {
          "type": "integer",
          "minimum": 1,
          "maximum": 65535
        },
        "network_read_timeout_ms": {
          "type": "integer",
          "minimum": 50,
          "maximum": 60000,
          "description": "Stream stall budget, not shutdown latency."
        },
        "network_max_frame_bytes": {
          "type": "integer",
          "minimum": 4096,
          "maximum": 67108864
        },
        "initial_roi_enabled": {
          "type": "boolean"
        },
        "initial_roi_normalized": {
          "type": "boolean"
        },
        "initial_roi": {
          "type": "array",
          "minItems": 4,
          "maxItems": 4,
          "items": {
            "type": "number"
          }
        }
      }
    },
    "stereo_epipolar": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "enabled": {
          "type": "boolean"
        },
        "soft_threshold_px": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "hard_threshold_px": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "min_confidence_floor": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "hard_mismatch_rejects_fresh_pair": {
          "type": "boolean"
        },
        "hard_mismatch_rejects_degraded_pair": {
          "type": "boolean"
        }
      }
    },
    "stereo_triangulation": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "min_single_camera_quality": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "min_stereo_reprojection_confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "single_camera_depth_confidence_scale": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "max_dlt_condition_number": {
          "type": "number",
          "minimum": 1
        },
        "min_dlt_strength_ratio": {
          "type": "number",
          "exclusiveMinimum": 0,
          "maximum": 1
        },
        "max_ray_closest_distance_m": {
          "type": "number",
          "minimum": 0
        },
        "min_ray_angle_deg": {
          "type": "number",
          "minimum": 0,
          "maximum": 90
        }
      }
    },
    "stereo_uncertainty": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "min_image_noise_sigma_px": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "min_lateral_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "min_depth_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "max_reported_position_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "max_conditioning_scale": {
          "type": "number",
          "minimum": 1
        }
      }
    },
    "solver_observation_weighting": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "reference_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "min_stddev_for_weight_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "max_weight_scale": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "fallback_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "temporal_process_variance_m2_per_s": {
          "type": "number",
          "minimum": 0
        },
        "max_temporal_process_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        }
      }
    },
    "stereo_identity": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "soft_threshold_px": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "hard_threshold_px": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "swap_ratio_margin": {
          "type": "number",
          "minimum": 1
        },
        "swap_absolute_margin": {
          "type": "number",
          "minimum": 0
        },
        "uncertainty_swap_margin_scale": {
          "type": "number",
          "minimum": 0
        },
        "partial_coverage_swap_margin": {
          "type": "number",
          "minimum": 0
        },
        "identity_prior_lateral_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "identity_prior_depth_stddev_m": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "identity_mahalanobis_score_scale": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "identity_max_mahalanobis_sq": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "identity_swap_nll_margin": {
          "type": "number",
          "minimum": 0
        },
        "strong_consistency_guard": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "min_assignment_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "min_detection_support": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "min_scored_lateral_pairs": {
          "type": "integer",
          "minimum": 1,
          "maximum": 6
        }
      }
    }
  }
})BTSCHEMA";

std::string JoinValidationErrors(const std::vector<std::string>& errors) {
    std::ostringstream out;
    for (std::size_t i = 0; i < errors.size(); ++i) {
        if (i > 0) {
            out << "; ";
        }
        out << errors[i];
    }
    return out.str();
}

std::string JsonPathForProperty(const std::string& base, const std::string& key) {
    return base == "$" ? "$." + key : base + "." + key;
}

std::string JsonPathForIndex(const std::string& base, std::size_t index) {
    return base + "[" + std::to_string(index) + "]";
}

bool JsonHasType(const nlohmann::json& value, const std::string& type) {
    if (type == "object") { return value.is_object(); }
    if (type == "array") { return value.is_array(); }
    if (type == "string") { return value.is_string(); }
    if (type == "boolean") { return value.is_boolean(); }
    if (type == "integer") { return value.is_number_integer(); }
    if (type == "number") { return value.is_number(); }
    if (type == "null") { return value.is_null(); }
    return true;
}

std::string DescribeAllowedType(const nlohmann::json& type_node) {
    if (type_node.is_string()) {
        return type_node.get<std::string>();
    }
    if (type_node.is_array()) {
        std::string out;
        for (const auto& entry : type_node) {
            if (!entry.is_string()) {
                continue;
            }
            if (!out.empty()) { out += "|"; }
            out += entry.get<std::string>();
        }
        return out.empty() ? "valid JSON type" : out;
    }
    return "valid JSON type";
}

bool JsonMatchesTypeNode(const nlohmann::json& value, const nlohmann::json& type_node) {
    if (type_node.is_string()) {
        return JsonHasType(value, type_node.get<std::string>());
    }
    if (type_node.is_array()) {
        for (const auto& entry : type_node) {
            if (entry.is_string() && JsonHasType(value, entry.get<std::string>())) {
                return true;
            }
        }
        return false;
    }
    return true;
}

const nlohmann::json* ResolveSchemaRef(const nlohmann::json& root_schema, const std::string& ref) {
    constexpr const char* prefix = "#/$defs/";
    if (ref.rfind(prefix, 0) != 0) {
        return nullptr;
    }
    const std::string key = ref.substr(std::char_traits<char>::length(prefix));
    if (!root_schema.contains("$defs") || !root_schema.at("$defs").is_object()) {
        return nullptr;
    }
    const auto& defs = root_schema.at("$defs");
    if (!defs.contains(key)) {
        return nullptr;
    }
    return &defs.at(key);
}

void ValidateJsonAgainstSchemaNode(
    const nlohmann::json& root_schema,
    const nlohmann::json& schema,
    const nlohmann::json& value,
    const std::string& path,
    std::vector<std::string>& errors) {

    if (!schema.is_object()) {
        return;
    }

    if (schema.contains("$ref") && schema.at("$ref").is_string()) {
        if (const nlohmann::json* resolved = ResolveSchemaRef(root_schema, schema.at("$ref").get<std::string>())) {
            ValidateJsonAgainstSchemaNode(root_schema, *resolved, value, path, errors);
        } else {
            errors.push_back(path + " schema has unresolved ref " + schema.at("$ref").get<std::string>());
        }
        return;
    }

    if (schema.contains("type") && !JsonMatchesTypeNode(value, schema.at("type"))) {
        errors.push_back(path + " must be " + DescribeAllowedType(schema.at("type")));
        return;
    }

    if (schema.contains("enum") && schema.at("enum").is_array()) {
        bool matched = false;
        for (const auto& allowed : schema.at("enum")) {
            if (value == allowed) {
                matched = true;
                break;
            }
        }
        if (!matched) {
            errors.push_back(path + " has value outside declared enum");
        }
    }

    if (value.is_number()) {
        const double numeric = value.get<double>();
        if (!std::isfinite(numeric)) {
            errors.push_back(path + " must be finite");
            return;
        }
        if (schema.contains("minimum") && numeric < schema.at("minimum").get<double>()) {
            errors.push_back(path + " must be >= " + std::to_string(schema.at("minimum").get<double>()));
        }
        if (schema.contains("maximum") && numeric > schema.at("maximum").get<double>()) {
            errors.push_back(path + " must be <= " + std::to_string(schema.at("maximum").get<double>()));
        }
        if (schema.contains("exclusiveMinimum") && numeric <= schema.at("exclusiveMinimum").get<double>()) {
            errors.push_back(path + " must be > " + std::to_string(schema.at("exclusiveMinimum").get<double>()));
        }
        if (schema.contains("exclusiveMaximum") && numeric >= schema.at("exclusiveMaximum").get<double>()) {
            errors.push_back(path + " must be < " + std::to_string(schema.at("exclusiveMaximum").get<double>()));
        }
    }

    if (value.is_array()) {
        if (schema.contains("minItems") && value.size() < schema.at("minItems").get<std::size_t>()) {
            errors.push_back(path + " must have at least " + std::to_string(schema.at("minItems").get<std::size_t>()) + " entries");
        }
        if (schema.contains("maxItems") && value.size() > schema.at("maxItems").get<std::size_t>()) {
            errors.push_back(path + " must have at most " + std::to_string(schema.at("maxItems").get<std::size_t>()) + " entries");
        }
        if (schema.contains("items")) {
            for (std::size_t i = 0; i < value.size(); ++i) {
                ValidateJsonAgainstSchemaNode(root_schema, schema.at("items"), value.at(i), JsonPathForIndex(path, i), errors);
            }
        }
    }

    if (value.is_object() && schema.contains("properties") && schema.at("properties").is_object()) {
        const auto& properties = schema.at("properties");
        for (auto it = properties.begin(); it != properties.end(); ++it) {
            if (value.contains(it.key())) {
                ValidateJsonAgainstSchemaNode(root_schema, it.value(), value.at(it.key()), JsonPathForProperty(path, it.key()), errors);
            }
        }
    }
}

nlohmann::json LoadBuiltinConfigSchema() {
    return nlohmann::json::parse(kBuiltinConfigSchemaJson);
}

std::vector<std::filesystem::path> ConfigSchemaCandidates(
    const nlohmann::json& config_json,
    const std::filesystem::path& config_path) {

    std::vector<std::filesystem::path> candidates;
    auto add = [&](std::filesystem::path p) {
        if (!p.empty()) {
            candidates.push_back(std::move(p));
        }
    };

    const auto parent = config_path.parent_path();
    if (config_json.is_object() && config_json.contains("$schema") && config_json.at("$schema").is_string()) {
        const std::string schema_ref = config_json.at("$schema").get<std::string>();
        if (schema_ref.find("://") == std::string::npos) {
            add(parent / schema_ref);
            add(parent / "config" / schema_ref);
        }
    }
    add(parent / "bodytracker-config.schema.json");
    add(parent / "config" / "bodytracker-config.schema.json");

    std::error_code ec;
    std::filesystem::path cursor = std::filesystem::current_path(ec);
    for (int depth = 0; !ec && depth < 6; ++depth) {
        add(cursor / "config" / "bodytracker-config.schema.json");
        if (!cursor.has_parent_path() || cursor.parent_path() == cursor) {
            break;
        }
        cursor = cursor.parent_path();
    }
    return candidates;
}

nlohmann::json LoadConfigSchemaForValidation(
    const nlohmann::json& config_json,
    const std::filesystem::path& config_path) {

    for (const auto& candidate : ConfigSchemaCandidates(config_json, config_path)) {
        std::error_code ec;
        if (!std::filesystem::exists(candidate, ec) || ec) {
            continue;
        }
        std::ifstream in(candidate);
        if (!in) {
            continue;
        }
        try {
            nlohmann::json schema;
            in >> schema;
            return schema;
        } catch (const std::exception&) {
            // Broken external schema must not make config validation nondeterministic;
            // the embedded copy below keeps the load-time contract authoritative.
            break;
        }
    }
    return LoadBuiltinConfigSchema();
}

Status ValidateRawConfigJsonAgainstSchema(
    const nlohmann::json& config_json,
    const std::filesystem::path& config_path) {

    try {
        const nlohmann::json schema = LoadConfigSchemaForValidation(config_json, config_path);
        std::vector<std::string> errors;
        ValidateJsonAgainstSchemaNode(schema, schema, config_json, "$", errors);
        if (!errors.empty()) {
            return Status::Error(StatusCode::ValidationError, "Config JSON schema validation failed: " + JoinValidationErrors(errors));
        }
    } catch (const std::exception& e) {
        return Status::Error(StatusCode::ValidationError, std::string("Config JSON schema validation failed: ") + e.what());
    }
    return Status::OK();
}

template <typename T>
T ReadOr(const nlohmann::json& j, const char* key, T fallback) {
    if (!j.is_object() || !j.contains(key)) {
        return fallback;
    }
    return j.at(key).get<T>();
}

Result<Rect2f> ReadRectOr(const nlohmann::json& j, const char* key, Rect2f fallback) {
    if (!j.is_object() || !j.contains(key)) {
        return fallback;
    }
    if (!j.at(key).is_array() || j.at(key).size() != 4) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must be an array of 4 numbers");
    }
    const auto& a = j.at(key);
    for (int i = 0; i < 4; ++i) {
        if (!a[i].is_number()) {
            return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must contain only numbers");
        }
    }
    Rect2f out{a[0].get<float>(), a[1].get<float>(), a[2].get<float>(), a[3].get<float>()};
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.width) || !std::isfinite(out.height)) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " contains a non-finite number");
    }
    return out;
}

Result<Vec3f> ReadVec3Or(const nlohmann::json& j, const char* key, Vec3f fallback) {
    if (!j.is_object() || !j.contains(key)) {
        return fallback;
    }
    if (!j.at(key).is_array() || j.at(key).size() != 3) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must be an array of 3 numbers");
    }
    const auto& a = j.at(key);
    if (!a[0].is_number() || !a[1].is_number() || !a[2].is_number()) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must contain only numbers");
    }
    Vec3f out{a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z)) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " contains a non-finite number");
    }
    return out;
}

Result<Quatf> ReadQuatOr(const nlohmann::json& j, const char* key, Quatf fallback) {
    if (!j.is_object() || !j.contains(key)) {
        return fallback;
    }
    if (!j.at(key).is_array() || j.at(key).size() != 4) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must be an array of 4 numbers");
    }
    const auto& a = j.at(key);
    if (!a[0].is_number() || !a[1].is_number() || !a[2].is_number() || !a[3].is_number()) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must contain only numbers");
    }
    Quatf out{a[0].get<float>(), a[1].get<float>(), a[2].get<float>(), a[3].get<float>()};
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z) || !std::isfinite(out.w)) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " contains a non-finite number");
    }
    const float len2 = out.x * out.x + out.y * out.y + out.z * out.z + out.w * out.w;
    if (!std::isfinite(len2) || len2 < 1e-12f) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must be a non-zero quaternion");
    }
    return Normalize(out);
}

Result<std::array<Vec3f, kOscTrackerRoleCount>> ReadTrackerRoleOffsetsOr(
    const nlohmann::json& j,
    const char* key,
    std::array<Vec3f, kOscTrackerRoleCount> fallback) {

    if (!j.is_object() || !j.contains(key)) {
        return fallback;
    }
    const auto& a = j.at(key);
    if (!a.is_array() || (a.size() != 5 && a.size() != kOscTrackerRoleCount)) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must be an array of 5 legacy or 8 current vec3 arrays");
    }

    auto read_vec = [&](std::size_t src_index) -> Result<Vec3f> {
        if (!a[src_index].is_array() || a[src_index].size() != 3 ||
            !a[src_index][0].is_number() || !a[src_index][1].is_number() || !a[src_index][2].is_number()) {
            return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " entries must be arrays of 3 numbers");
        }
        Vec3f v{a[src_index][0].get<float>(), a[src_index][1].get<float>(), a[src_index][2].get<float>()};
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
            return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " contains a non-finite number");
        }
        return v;
    };

    if (a.size() == kOscTrackerRoleCount) {
        for (std::size_t i = 0; i < kOscTrackerRoleCount; ++i) {
            auto value = read_vec(i);
            if (!value.ok()) {
                return value.status();
            }
            fallback[i] = value.value();
        }
        return fallback;
    }

    // Legacy order was [pelvis, left_foot, right_foot, left_knee, right_knee].
    // Current order is [pelvis, left_foot, right_foot, chest, left_elbow, right_elbow, left_knee, right_knee].
    const std::array<std::size_t, 5> legacy_to_current{0, 1, 2, 6, 7};
    for (std::size_t legacy_index = 0; legacy_index < legacy_to_current.size(); ++legacy_index) {
        auto value = read_vec(legacy_index);
        if (!value.ok()) {
            return value.status();
        }
        fallback[legacy_to_current[legacy_index]] = value.value();
    }
    return fallback;
}

Result<std::array<float, 9>> ReadFloatArray9Or(const nlohmann::json& j, const char* key, std::array<float, 9> fallback) {
    if (!j.is_object() || !j.contains(key)) {
        return fallback;
    }
    if (!j.at(key).is_array() || j.at(key).size() != fallback.size()) {
        return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must be an array of 9 numbers");
    }
    const auto& a = j.at(key);
    for (std::size_t i = 0; i < fallback.size(); ++i) {
        if (!a[i].is_number()) {
            return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " must contain only numbers");
        }
        fallback[i] = a[i].get<float>();
        if (!std::isfinite(fallback[i])) {
            return Status::Error(StatusCode::ValidationError, std::string("Config key ") + key + " contains a non-finite number");
        }
    }
    return fallback;
}

Status ReadCameraConfig(const nlohmann::json& j, CameraConfig& cfg) {
    cfg.source = ReadOr<std::string>(j, "source", cfg.source);
    cfg.device_index = ReadOr<int>(j, "device_index", cfg.device_index);
    cfg.width = ReadOr<int>(j, "width", cfg.width);
    cfg.height = ReadOr<int>(j, "height", cfg.height);
    cfg.fps = ReadOr<int>(j, "fps", cfg.fps);
    cfg.network_bind_address = ReadOr<std::string>(j, "network_bind_address", cfg.network_bind_address);
    cfg.network_port = ReadOr<int>(j, "network_port", cfg.network_port);
    cfg.network_read_timeout_ms = ReadOr<int>(j, "network_read_timeout_ms", cfg.network_read_timeout_ms);
    cfg.network_max_frame_bytes = ReadOr<int>(j, "network_max_frame_bytes", cfg.network_max_frame_bytes);
    cfg.initial_roi_enabled = ReadOr<bool>(j, "initial_roi_enabled", cfg.initial_roi_enabled);
    cfg.initial_roi_normalized = ReadOr<bool>(j, "initial_roi_normalized", cfg.initial_roi_normalized);
    const auto roi = ReadRectOr(j, "initial_roi", cfg.initial_roi);
    if (!roi.ok()) {
        return roi.status();
    }
    cfg.initial_roi = roi.value();
    return Status::OK();
}

Status ReadStereoEpipolarConfig(const nlohmann::json& j, StereoEpipolarConfig& cfg) {
    cfg.enabled = ReadOr<bool>(j, "enabled", cfg.enabled);
    cfg.soft_threshold_px = ReadOr<float>(j, "soft_threshold_px", cfg.soft_threshold_px);
    cfg.hard_threshold_px = ReadOr<float>(j, "hard_threshold_px", cfg.hard_threshold_px);
    cfg.min_confidence_floor = ReadOr<float>(j, "min_confidence_floor", cfg.min_confidence_floor);
    cfg.hard_mismatch_rejects_fresh_pair = ReadOr<bool>(j, "hard_mismatch_rejects_fresh_pair", cfg.hard_mismatch_rejects_fresh_pair);
    cfg.hard_mismatch_rejects_degraded_pair = ReadOr<bool>(j, "hard_mismatch_rejects_degraded_pair", cfg.hard_mismatch_rejects_degraded_pair);
    return Status::OK();
}

Status ReadStereoTriangulationConfig(const nlohmann::json& j, StereoTriangulationConfig& cfg) {
    cfg.min_single_camera_quality = ReadOr<float>(j, "min_single_camera_quality", cfg.min_single_camera_quality);
    cfg.min_stereo_reprojection_confidence = ReadOr<float>(j, "min_stereo_reprojection_confidence", cfg.min_stereo_reprojection_confidence);
    cfg.single_camera_depth_confidence_scale = ReadOr<float>(j, "single_camera_depth_confidence_scale", cfg.single_camera_depth_confidence_scale);
    cfg.max_dlt_condition_number = ReadOr<float>(j, "max_dlt_condition_number", cfg.max_dlt_condition_number);
    cfg.min_dlt_strength_ratio = ReadOr<float>(j, "min_dlt_strength_ratio", cfg.min_dlt_strength_ratio);
    cfg.max_ray_closest_distance_m = ReadOr<float>(j, "max_ray_closest_distance_m", cfg.max_ray_closest_distance_m);
    cfg.min_ray_angle_deg = ReadOr<float>(j, "min_ray_angle_deg", cfg.min_ray_angle_deg);
    return Status::OK();
}

Status ReadStereoMeasurementUncertaintyConfig(const nlohmann::json& j, StereoMeasurementUncertaintyConfig& cfg) {
    cfg.min_image_noise_sigma_px = ReadOr<float>(j, "min_image_noise_sigma_px", cfg.min_image_noise_sigma_px);
    cfg.min_lateral_stddev_m = ReadOr<float>(j, "min_lateral_stddev_m", cfg.min_lateral_stddev_m);
    cfg.min_depth_stddev_m = ReadOr<float>(j, "min_depth_stddev_m", cfg.min_depth_stddev_m);
    cfg.max_reported_position_stddev_m = ReadOr<float>(j, "max_reported_position_stddev_m", cfg.max_reported_position_stddev_m);
    cfg.max_conditioning_scale = ReadOr<float>(j, "max_conditioning_scale", cfg.max_conditioning_scale);
    return Status::OK();
}

Status ReadSolverObservationWeightingConfig(const nlohmann::json& j, SolverObservationWeightingConfig& cfg) {
    cfg.reference_stddev_m = ReadOr<float>(j, "reference_stddev_m", cfg.reference_stddev_m);
    cfg.min_stddev_for_weight_m = ReadOr<float>(j, "min_stddev_for_weight_m", cfg.min_stddev_for_weight_m);
    cfg.max_weight_scale = ReadOr<float>(j, "max_weight_scale", cfg.max_weight_scale);
    cfg.fallback_stddev_m = ReadOr<float>(j, "fallback_stddev_m", cfg.fallback_stddev_m);
    cfg.temporal_process_variance_m2_per_s = ReadOr<float>(j, "temporal_process_variance_m2_per_s", cfg.temporal_process_variance_m2_per_s);
    cfg.max_temporal_process_stddev_m = ReadOr<float>(j, "max_temporal_process_stddev_m", cfg.max_temporal_process_stddev_m);
    return Status::OK();
}

Status ReadStereoIdentityEpipolarConfig(const nlohmann::json& j, StereoIdentityEpipolarConfig& cfg) {
    cfg.soft_threshold_px = ReadOr<float>(j, "soft_threshold_px", cfg.soft_threshold_px);
    cfg.hard_threshold_px = ReadOr<float>(j, "hard_threshold_px", cfg.hard_threshold_px);
    cfg.swap_ratio_margin = ReadOr<float>(j, "swap_ratio_margin", cfg.swap_ratio_margin);
    cfg.swap_absolute_margin = ReadOr<float>(j, "swap_absolute_margin", cfg.swap_absolute_margin);
    cfg.uncertainty_swap_margin_scale = ReadOr<float>(j, "uncertainty_swap_margin_scale", cfg.uncertainty_swap_margin_scale);
    cfg.partial_coverage_swap_margin = ReadOr<float>(j, "partial_coverage_swap_margin", cfg.partial_coverage_swap_margin);
    cfg.identity_prior_lateral_stddev_m = ReadOr<float>(j, "identity_prior_lateral_stddev_m", cfg.identity_prior_lateral_stddev_m);
    cfg.identity_prior_depth_stddev_m = ReadOr<float>(j, "identity_prior_depth_stddev_m", cfg.identity_prior_depth_stddev_m);
    cfg.identity_mahalanobis_score_scale = ReadOr<float>(j, "identity_mahalanobis_score_scale", cfg.identity_mahalanobis_score_scale);
    cfg.identity_max_mahalanobis_sq = ReadOr<float>(j, "identity_max_mahalanobis_sq", cfg.identity_max_mahalanobis_sq);
    cfg.identity_swap_nll_margin = ReadOr<float>(j, "identity_swap_nll_margin", cfg.identity_swap_nll_margin);
    cfg.strong_consistency_guard = ReadOr<float>(j, "strong_consistency_guard", cfg.strong_consistency_guard);
    cfg.min_assignment_score = ReadOr<float>(j, "min_assignment_score", cfg.min_assignment_score);
    cfg.min_detection_support = ReadOr<float>(j, "min_detection_support", cfg.min_detection_support);
    cfg.min_scored_lateral_pairs = ReadOr<int>(j, "min_scored_lateral_pairs", cfg.min_scored_lateral_pairs);
    return Status::OK();
}

Status ReadMotionConsistencyConfig(const nlohmann::json& j, MotionConsistencyConfig& cfg) {
    cfg.enabled = ReadOr<bool>(j, "enabled", cfg.enabled);
    cfg.confirm_frames = ReadOr<int>(j, "confirm_frames", cfg.confirm_frames);
    cfg.min_motion_m = ReadOr<float>(j, "min_motion_m", cfg.min_motion_m);
    cfg.stationary_deadzone_m = ReadOr<float>(j, "stationary_deadzone_m", cfg.stationary_deadzone_m);
    cfg.max_direction_deviation_deg = ReadOr<float>(j, "max_direction_deviation_deg", cfg.max_direction_deviation_deg);
    cfg.max_lateral_deviation_ratio = ReadOr<float>(j, "max_lateral_deviation_ratio", cfg.max_lateral_deviation_ratio);
    cfg.max_speed_change_ratio = ReadOr<float>(j, "max_speed_change_ratio", cfg.max_speed_change_ratio);
    cfg.reject_confidence_decay_per_second = ReadOr<float>(j, "reject_confidence_decay_per_second", cfg.reject_confidence_decay_per_second);
    cfg.planted_foot_max_drift_m = ReadOr<float>(j, "planted_foot_max_drift_m", cfg.planted_foot_max_drift_m);
    cfg.planted_foot_release_confirm_frames = ReadOr<int>(j, "planted_foot_release_confirm_frames", cfg.planted_foot_release_confirm_frames);
    cfg.contact_root_correction_gain = ReadOr<float>(j, "contact_root_correction_gain", cfg.contact_root_correction_gain);
    cfg.contact_root_max_correction_m = ReadOr<float>(j, "contact_root_max_correction_m", cfg.contact_root_max_correction_m);
    cfg.contact_root_max_residual_m = ReadOr<float>(j, "contact_root_max_residual_m", cfg.contact_root_max_residual_m);
    cfg.contact_root_max_disagreement_m = ReadOr<float>(j, "contact_root_max_disagreement_m", cfg.contact_root_max_disagreement_m);
    cfg.contact_root_min_alignment = ReadOr<float>(j, "contact_root_min_alignment", cfg.contact_root_min_alignment);
    cfg.contact_root_min_support_confidence = ReadOr<float>(j, "contact_root_min_support_confidence", cfg.contact_root_min_support_confidence);
    cfg.root_max_speed_mps = ReadOr<float>(j, "root_max_speed_mps", cfg.root_max_speed_mps);
    cfg.foot_max_speed_mps = ReadOr<float>(j, "foot_max_speed_mps", cfg.foot_max_speed_mps);
    cfg.root_max_accel_mps2 = ReadOr<float>(j, "root_max_accel_mps2", cfg.root_max_accel_mps2);
    cfg.foot_max_accel_mps2 = ReadOr<float>(j, "foot_max_accel_mps2", cfg.foot_max_accel_mps2);
    cfg.confirm_scale_m = ReadOr<float>(j, "confirm_scale_m", cfg.confirm_scale_m);
    cfg.confirm_frames_max = ReadOr<int>(j, "confirm_frames_max", cfg.confirm_frames_max);
    cfg.one_euro_enabled = ReadOr<bool>(j, "one_euro_enabled", cfg.one_euro_enabled);
    cfg.one_euro_min_cutoff_hz = ReadOr<float>(j, "one_euro_min_cutoff_hz", cfg.one_euro_min_cutoff_hz);
    cfg.one_euro_beta = ReadOr<float>(j, "one_euro_beta", cfg.one_euro_beta);
    cfg.one_euro_d_cutoff_hz = ReadOr<float>(j, "one_euro_d_cutoff_hz", cfg.one_euro_d_cutoff_hz);
    return Status::OK();
}

Status ReadTrackerEkfConfig(const nlohmann::json& j, TrackerEkfConfig& cfg) {
    cfg.enabled = ReadOr<bool>(j, "enabled", cfg.enabled);
    cfg.process_noise_mps2 = ReadOr<float>(j, "process_noise_mps2", cfg.process_noise_mps2);
    cfg.min_measurement_variance_m2 = ReadOr<float>(j, "min_measurement_variance_m2", cfg.min_measurement_variance_m2);
    cfg.max_measurement_variance_m2 = ReadOr<float>(j, "max_measurement_variance_m2", cfg.max_measurement_variance_m2);
    cfg.support_variance_scale = ReadOr<float>(j, "support_variance_scale", cfg.support_variance_scale);
    cfg.missing_velocity_decay = ReadOr<float>(j, "missing_velocity_decay", cfg.missing_velocity_decay);
    cfg.foot_orientation_gain = ReadOr<float>(j, "foot_orientation_gain", cfg.foot_orientation_gain);
    cfg.mahalanobis_gate_enabled = ReadOr<bool>(j, "mahalanobis_gate_enabled", cfg.mahalanobis_gate_enabled);
    cfg.mahalanobis_gate_chi2 = ReadOr<float>(j, "mahalanobis_gate_chi2", cfg.mahalanobis_gate_chi2);
    cfg.outlier_variance_scale = ReadOr<float>(j, "outlier_variance_scale", cfg.outlier_variance_scale);
    return Status::OK();
}

Status ReadTemporalUpdateConfig(const nlohmann::json& j, TemporalUpdateConfig& cfg) {
    cfg.free_gain = ReadOr<float>(j, "free_gain", cfg.free_gain);
    cfg.supported_gain = ReadOr<float>(j, "supported_gain", cfg.supported_gain);
    cfg.foot_free_gain = ReadOr<float>(j, "foot_free_gain", cfg.foot_free_gain);
    cfg.foot_supported_gain = ReadOr<float>(j, "foot_supported_gain", cfg.foot_supported_gain);
    return Status::OK();
}

Status ReadBodyCalibrationModeConfig(const nlohmann::json& j, BodyCalibrationModeConfig& cfg) {
    cfg.enabled = ReadOr<bool>(j, "enabled", cfg.enabled);
    cfg.auto_persist = ReadOr<bool>(j, "auto_persist", cfg.auto_persist);
    cfg.required_seconds = ReadOr<float>(j, "required_seconds", cfg.required_seconds);
    cfg.min_overall_confidence = ReadOr<float>(j, "min_overall_confidence", cfg.min_overall_confidence);
    cfg.max_segment_cv = ReadOr<float>(j, "max_segment_cv", cfg.max_segment_cv);
    return Status::OK();
}

Result<TrackingMode> ReadTrackingMode(const nlohmann::json& tracking, TrackingMode fallback) {
    const std::string value = ReadOr<std::string>(tracking, "mode", ToString(fallback));
    if (value == "stereo") {
        return TrackingMode::Stereo;
    }
    if (value == "monocular") {
        return TrackingMode::Monocular;
    }
    return Status::Error(StatusCode::ValidationError, "tracking.mode must be either stereo or monocular");
}

Status ReadMonocularTrackingConfig(const nlohmann::json& j, MonocularTrackingConfig& cfg) {
    cfg.image_width = ReadOr<int>(j, "image_width", cfg.image_width);
    cfg.image_height = ReadOr<int>(j, "image_height", cfg.image_height);
    cfg.horizontal_fov_deg = ReadOr<float>(j, "horizontal_fov_deg", cfg.horizontal_fov_deg);
    cfg.user_height_m = ReadOr<float>(j, "user_height_m", cfg.user_height_m);
    cfg.camera_height_m = ReadOr<float>(j, "camera_height_m", cfg.camera_height_m);
    cfg.default_depth_m = ReadOr<float>(j, "default_depth_m", cfg.default_depth_m);
    cfg.depth_confidence_scale = ReadOr<float>(j, "depth_confidence_scale", cfg.depth_confidence_scale);
    cfg.min_keypoint_confidence = ReadOr<float>(j, "min_keypoint_confidence", cfg.min_keypoint_confidence);
    cfg.min_seed_count = ReadOr<int>(j, "min_seed_count", cfg.min_seed_count);
    cfg.floor_scale_assist_enabled = ReadOr<bool>(j, "floor_scale_assist_enabled", cfg.floor_scale_assist_enabled);
    cfg.floor_geometry_calibration_enabled = ReadOr<bool>(j, "floor_geometry_calibration_enabled", cfg.floor_geometry_calibration_enabled);
    cfg.floor_geometry_type = ReadOr<std::string>(j, "floor_geometry_type", cfg.floor_geometry_type);
    cfg.floor_depth_line_spacing_m = ReadOr<float>(j, "floor_depth_line_spacing_m", cfg.floor_depth_line_spacing_m);
    cfg.floor_depth_line_spacing_px = ReadOr<float>(j, "floor_depth_line_spacing_px", cfg.floor_depth_line_spacing_px);
    cfg.floor_depth_reference_y_px = ReadOr<float>(j, "floor_depth_reference_y_px", cfg.floor_depth_reference_y_px);
    cfg.floor_depth_reference_m = ReadOr<float>(j, "floor_depth_reference_m", cfg.floor_depth_reference_m);
    cfg.floor_depth_confidence = ReadOr<float>(j, "floor_depth_confidence", cfg.floor_depth_confidence);
    cfg.floor_second_axis_spacing_m = ReadOr<float>(j, "floor_second_axis_spacing_m", cfg.floor_second_axis_spacing_m);
    cfg.floor_geometry_confidence = ReadOr<float>(j, "floor_geometry_confidence", cfg.floor_geometry_confidence);
    cfg.floor_projective_homography_enabled = ReadOr<bool>(j, "floor_projective_homography_enabled", cfg.floor_projective_homography_enabled);
    const auto floor_from_image = ReadFloatArray9Or(j, "floor_from_image", cfg.floor_from_image);
    if (!floor_from_image.ok()) {
        return floor_from_image.status();
    }
    cfg.floor_from_image = floor_from_image.value();
    const auto image_from_floor = ReadFloatArray9Or(j, "image_from_floor", cfg.image_from_floor);
    if (!image_from_floor.ok()) {
        return image_from_floor.status();
    }
    cfg.image_from_floor = image_from_floor.value();
    cfg.floor_projective_confidence = ReadOr<float>(j, "floor_projective_confidence", cfg.floor_projective_confidence);
    cfg.floor_distortion_correction_enabled = ReadOr<bool>(j, "floor_distortion_correction_enabled", cfg.floor_distortion_correction_enabled);
    cfg.floor_distortion_confidence = ReadOr<float>(j, "floor_distortion_confidence", cfg.floor_distortion_confidence);
    cfg.floor_radial_k1 = ReadOr<float>(j, "floor_radial_k1", cfg.floor_radial_k1);
    cfg.floor_radial_k2 = ReadOr<float>(j, "floor_radial_k2", cfg.floor_radial_k2);
    cfg.floor_tangential_p1 = ReadOr<float>(j, "floor_tangential_p1", cfg.floor_tangential_p1);
    cfg.floor_tangential_p2 = ReadOr<float>(j, "floor_tangential_p2", cfg.floor_tangential_p2);
    cfg.floor_camera_orientation_enabled = ReadOr<bool>(j, "floor_camera_orientation_enabled", cfg.floor_camera_orientation_enabled);
    cfg.floor_camera_pitch_rad = ReadOr<float>(j, "floor_camera_pitch_rad", cfg.floor_camera_pitch_rad);
    cfg.floor_camera_roll_rad = ReadOr<float>(j, "floor_camera_roll_rad", cfg.floor_camera_roll_rad);
    cfg.floor_camera_orientation_confidence = ReadOr<float>(j, "floor_camera_orientation_confidence", cfg.floor_camera_orientation_confidence);
    return Status::OK();
}

Status ValidateMonocularTrackingConfig(const MonocularTrackingConfig& cfg) {
    if (cfg.image_width <= 0 || cfg.image_height <= 0) {
        return Status::Error(StatusCode::ValidationError, "tracking.monocular image_width/image_height must be positive");
    }

    // These values are required to build a finite single-camera projection
    // profile. Optional floor-scale assist fields are validated for finite
    // ranges below, but an incomplete assist must not make the whole config
    // unloadable: the projector simply falls back to body/default/floor-ray
    // monocular depth and the UI/preflight report the assist as weak/inactive/invalid.
    if (!std::isfinite(cfg.horizontal_fov_deg) ||
        cfg.horizontal_fov_deg < tracking_constants::kMonocularMinFovDeg ||
        cfg.horizontal_fov_deg > tracking_constants::kMonocularMaxFovDeg ||
        !std::isfinite(cfg.user_height_m) || cfg.user_height_m < 0.8f || cfg.user_height_m > 2.5f ||
        !std::isfinite(cfg.camera_height_m) || cfg.camera_height_m < 0.1f || cfg.camera_height_m > 3.5f ||
        !std::isfinite(cfg.default_depth_m) ||
        cfg.default_depth_m < tracking_constants::kMonocularMinDepthM ||
        cfg.default_depth_m > tracking_constants::kMonocularMaxDepthM ||
        !std::isfinite(cfg.depth_confidence_scale) || cfg.depth_confidence_scale <= 0.0f || cfg.depth_confidence_scale >= 1.0f ||
        !std::isfinite(cfg.min_keypoint_confidence) || cfg.min_keypoint_confidence < 0.0f || cfg.min_keypoint_confidence > 1.0f ||
        !std::isfinite(cfg.floor_depth_line_spacing_m) || cfg.floor_depth_line_spacing_m < 0.0f || cfg.floor_depth_line_spacing_m > 2.0f ||
        !std::isfinite(cfg.floor_depth_line_spacing_px) || cfg.floor_depth_line_spacing_px < 0.0f || cfg.floor_depth_line_spacing_px > 2000.0f ||
        !std::isfinite(cfg.floor_depth_reference_y_px) || cfg.floor_depth_reference_y_px < 0.0f ||
        !std::isfinite(cfg.floor_depth_reference_m) || cfg.floor_depth_reference_m < 0.0f || cfg.floor_depth_reference_m > 8.0f ||
        !std::isfinite(cfg.floor_depth_confidence) || cfg.floor_depth_confidence < 0.0f || cfg.floor_depth_confidence > 1.0f ||
        !std::isfinite(cfg.floor_second_axis_spacing_m) || cfg.floor_second_axis_spacing_m < 0.0f || cfg.floor_second_axis_spacing_m > 2.0f ||
        !std::isfinite(cfg.floor_geometry_confidence) || cfg.floor_geometry_confidence < 0.0f || cfg.floor_geometry_confidence > 1.0f ||
        !std::isfinite(cfg.floor_distortion_confidence) || cfg.floor_distortion_confidence < 0.0f || cfg.floor_distortion_confidence > 1.0f ||
        !std::isfinite(cfg.floor_camera_orientation_confidence) || cfg.floor_camera_orientation_confidence < 0.0f || cfg.floor_camera_orientation_confidence > 1.0f ||
        !std::isfinite(cfg.floor_camera_pitch_rad) || std::abs(cfg.floor_camera_pitch_rad) > 1.4f ||
        !std::isfinite(cfg.floor_camera_roll_rad) || std::abs(cfg.floor_camera_roll_rad) > 1.4f ||
        !std::isfinite(cfg.floor_radial_k1) || !std::isfinite(cfg.floor_radial_k2) ||
        !std::isfinite(cfg.floor_tangential_p1) || !std::isfinite(cfg.floor_tangential_p2)) {
        return Status::Error(StatusCode::ValidationError, "tracking.monocular values must be finite and in valid markerless ranges");
    }
    if (cfg.min_seed_count < 1 || cfg.min_seed_count > static_cast<int>(kHalpe26Count)) {
        return Status::Error(StatusCode::ValidationError, "tracking.monocular.min_seed_count must be between 1 and 26");
    }
    const auto matrix_finite = [](const std::array<float, 9>& m) {
        return std::all_of(m.begin(), m.end(), [](float v) { return std::isfinite(v); });
    };
    const auto matrix_det = [](const std::array<float, 9>& m) {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    };
    if (!matrix_finite(cfg.floor_from_image) || !matrix_finite(cfg.image_from_floor) ||
        !std::isfinite(cfg.floor_projective_confidence) ||
        cfg.floor_projective_confidence < 0.0f || cfg.floor_projective_confidence > 1.0f) {
        return Status::Error(StatusCode::ValidationError, "tracking.monocular floor homography values must be finite with confidence in 0..1");
    }
    if (cfg.floor_projective_homography_enabled &&
        (std::abs(matrix_det(cfg.floor_from_image)) < 1e-8f ||
         std::abs(matrix_det(cfg.image_from_floor)) < 1e-8f)) {
        return Status::Error(StatusCode::ValidationError, "tracking.monocular floor homography is enabled but contains a singular transform");
    }
    return Status::OK();
}

Status ValidateStereoEpipolarConfig(const StereoEpipolarConfig& cfg) {
    if (!std::isfinite(cfg.soft_threshold_px) || cfg.soft_threshold_px <= 0.0f ||
        !std::isfinite(cfg.hard_threshold_px) || cfg.hard_threshold_px <= 0.0f ||
        cfg.soft_threshold_px > cfg.hard_threshold_px ||
        !std::isfinite(cfg.min_confidence_floor) || cfg.min_confidence_floor < 0.0f || cfg.min_confidence_floor > 1.0f) {
        return Status::Error(StatusCode::ValidationError, "stereo_epipolar thresholds must be finite and in valid ranges");
    }
    return Status::OK();
}

Status ValidateStereoTriangulationConfig(const StereoTriangulationConfig& cfg) {
    if (!std::isfinite(cfg.min_single_camera_quality) || cfg.min_single_camera_quality < 0.0f || cfg.min_single_camera_quality > 1.0f ||
        !std::isfinite(cfg.min_stereo_reprojection_confidence) || cfg.min_stereo_reprojection_confidence < 0.0f || cfg.min_stereo_reprojection_confidence > 1.0f ||
        !std::isfinite(cfg.single_camera_depth_confidence_scale) || cfg.single_camera_depth_confidence_scale < 0.0f || cfg.single_camera_depth_confidence_scale > 1.0f ||
        !std::isfinite(cfg.max_dlt_condition_number) || cfg.max_dlt_condition_number < 1.0f ||
        !std::isfinite(cfg.min_dlt_strength_ratio) || cfg.min_dlt_strength_ratio <= 0.0f || cfg.min_dlt_strength_ratio > 1.0f ||
        !std::isfinite(cfg.max_ray_closest_distance_m) || cfg.max_ray_closest_distance_m < 0.0f ||
        !std::isfinite(cfg.min_ray_angle_deg) || cfg.min_ray_angle_deg < 0.0f || cfg.min_ray_angle_deg > 90.0f) {
        return Status::Error(StatusCode::ValidationError, "stereo_triangulation values must be finite and in valid ranges");
    }
    return Status::OK();
}

Status ValidateStereoMeasurementUncertaintyConfig(const StereoMeasurementUncertaintyConfig& cfg) {
    if (!std::isfinite(cfg.min_image_noise_sigma_px) || cfg.min_image_noise_sigma_px <= 0.0f ||
        !std::isfinite(cfg.min_lateral_stddev_m) || cfg.min_lateral_stddev_m <= 0.0f ||
        !std::isfinite(cfg.min_depth_stddev_m) || cfg.min_depth_stddev_m <= 0.0f ||
        !std::isfinite(cfg.max_reported_position_stddev_m) || cfg.max_reported_position_stddev_m <= 0.0f ||
        !std::isfinite(cfg.max_conditioning_scale) || cfg.max_conditioning_scale < 1.0f) {
        return Status::Error(StatusCode::ValidationError, "stereo_uncertainty values must be finite and positive");
    }
    return Status::OK();
}

Status ValidateSolverObservationWeightingConfig(const SolverObservationWeightingConfig& cfg) {
    if (!std::isfinite(cfg.reference_stddev_m) || cfg.reference_stddev_m <= 0.0f ||
        !std::isfinite(cfg.min_stddev_for_weight_m) || cfg.min_stddev_for_weight_m <= 0.0f ||
        !std::isfinite(cfg.max_weight_scale) || cfg.max_weight_scale <= 0.0f ||
        !std::isfinite(cfg.fallback_stddev_m) || cfg.fallback_stddev_m <= 0.0f ||
        !std::isfinite(cfg.temporal_process_variance_m2_per_s) || cfg.temporal_process_variance_m2_per_s < 0.0f ||
        !std::isfinite(cfg.max_temporal_process_stddev_m) || cfg.max_temporal_process_stddev_m <= 0.0f) {
        return Status::Error(StatusCode::ValidationError, "solver_observation_weighting values must be finite and positive");
    }
    return Status::OK();
}

Status ValidateStereoIdentityEpipolarConfig(const StereoIdentityEpipolarConfig& cfg) {
    if (!std::isfinite(cfg.soft_threshold_px) || cfg.soft_threshold_px <= 0.0f ||
        !std::isfinite(cfg.hard_threshold_px) || cfg.hard_threshold_px <= 0.0f ||
        cfg.soft_threshold_px > cfg.hard_threshold_px ||
        !std::isfinite(cfg.swap_ratio_margin) || cfg.swap_ratio_margin < 1.0f ||
        !std::isfinite(cfg.swap_absolute_margin) || cfg.swap_absolute_margin < 0.0f ||
        !std::isfinite(cfg.uncertainty_swap_margin_scale) || cfg.uncertainty_swap_margin_scale < 0.0f ||
        !std::isfinite(cfg.partial_coverage_swap_margin) || cfg.partial_coverage_swap_margin < 0.0f ||
        !std::isfinite(cfg.identity_prior_lateral_stddev_m) || cfg.identity_prior_lateral_stddev_m <= 0.0f ||
        !std::isfinite(cfg.identity_prior_depth_stddev_m) || cfg.identity_prior_depth_stddev_m <= 0.0f ||
        !std::isfinite(cfg.identity_mahalanobis_score_scale) || cfg.identity_mahalanobis_score_scale <= 0.0f ||
        !std::isfinite(cfg.identity_max_mahalanobis_sq) || cfg.identity_max_mahalanobis_sq <= 0.0f ||
        !std::isfinite(cfg.identity_swap_nll_margin) || cfg.identity_swap_nll_margin < 0.0f ||
        !std::isfinite(cfg.strong_consistency_guard) || cfg.strong_consistency_guard < 0.0f || cfg.strong_consistency_guard > 1.0f ||
        !std::isfinite(cfg.min_assignment_score) || cfg.min_assignment_score < 0.0f || cfg.min_assignment_score > 1.0f ||
        !std::isfinite(cfg.min_detection_support) || cfg.min_detection_support < 0.0f || cfg.min_detection_support > 1.0f ||
        cfg.min_scored_lateral_pairs < 1 || cfg.min_scored_lateral_pairs > 6) {
        return Status::Error(StatusCode::ValidationError, "stereo_identity values must be finite and in valid ranges");
    }
    return Status::OK();
}

Status ValidateMotionConsistencyConfig(const MotionConsistencyConfig& cfg) {
    if (cfg.confirm_frames < 1 || cfg.confirm_frames > 8) {
        return Status::Error(StatusCode::ValidationError, "motion_consistency confirm_frames must be between 1 and 8");
    }
    if (cfg.planted_foot_release_confirm_frames < 1 || cfg.planted_foot_release_confirm_frames > 8) {
        return Status::Error(StatusCode::ValidationError, "motion_consistency planted_foot_release_confirm_frames must be between 1 and 8");
    }
    if (!std::isfinite(cfg.min_motion_m) || cfg.min_motion_m < 0.0f ||
        !std::isfinite(cfg.stationary_deadzone_m) || cfg.stationary_deadzone_m < 0.0f ||
        !std::isfinite(cfg.max_direction_deviation_deg) || cfg.max_direction_deviation_deg <= 0.0f || cfg.max_direction_deviation_deg > 180.0f ||
        !std::isfinite(cfg.max_lateral_deviation_ratio) || cfg.max_lateral_deviation_ratio < 0.0f ||
        !std::isfinite(cfg.max_speed_change_ratio) || cfg.max_speed_change_ratio < 1.0f ||
        !std::isfinite(cfg.reject_confidence_decay_per_second) || cfg.reject_confidence_decay_per_second < 0.0f ||
        !std::isfinite(cfg.planted_foot_max_drift_m) || cfg.planted_foot_max_drift_m < 0.0f ||
        !std::isfinite(cfg.contact_root_correction_gain) || cfg.contact_root_correction_gain < 0.0f || cfg.contact_root_correction_gain > 1.0f ||
        !std::isfinite(cfg.contact_root_max_correction_m) || cfg.contact_root_max_correction_m < 0.0f ||
        !std::isfinite(cfg.contact_root_max_residual_m) || cfg.contact_root_max_residual_m < 0.0f ||
        !std::isfinite(cfg.contact_root_max_disagreement_m) || cfg.contact_root_max_disagreement_m < 0.0f ||
        !std::isfinite(cfg.contact_root_min_alignment) || cfg.contact_root_min_alignment < -1.0f || cfg.contact_root_min_alignment > 1.0f ||
        !std::isfinite(cfg.contact_root_min_support_confidence) || cfg.contact_root_min_support_confidence < 0.0f || cfg.contact_root_min_support_confidence > 1.0f) {
        return Status::Error(StatusCode::ValidationError, "motion_consistency thresholds must be finite and in valid ranges");
    }
    return Status::OK();
}

Status ValidateTrackerEkfConfig(const TrackerEkfConfig& cfg) {
    if (!std::isfinite(cfg.process_noise_mps2) || cfg.process_noise_mps2 < 0.0f ||
        !std::isfinite(cfg.min_measurement_variance_m2) || cfg.min_measurement_variance_m2 <= 0.0f ||
        !std::isfinite(cfg.max_measurement_variance_m2) || cfg.max_measurement_variance_m2 <= 0.0f ||
        cfg.min_measurement_variance_m2 > cfg.max_measurement_variance_m2 ||
        !std::isfinite(cfg.support_variance_scale) || cfg.support_variance_scale <= 0.0f || cfg.support_variance_scale > 1.0f ||
        !std::isfinite(cfg.missing_velocity_decay) || cfg.missing_velocity_decay < 0.0f || cfg.missing_velocity_decay > 1.0f ||
        !std::isfinite(cfg.foot_orientation_gain) || cfg.foot_orientation_gain < 0.0f || cfg.foot_orientation_gain > 1.0f) {
        return Status::Error(StatusCode::ValidationError, "tracker_ekf thresholds must be finite and in valid ranges");
    }
    return Status::OK();
}

Status ValidateTemporalUpdateConfig(const TemporalUpdateConfig& cfg) {
    if (!std::isfinite(cfg.free_gain) || cfg.free_gain < 0.0f || cfg.free_gain > 1.0f ||
        !std::isfinite(cfg.supported_gain) || cfg.supported_gain < 0.0f || cfg.supported_gain > 1.0f ||
        !std::isfinite(cfg.foot_free_gain) || cfg.foot_free_gain < 0.0f || cfg.foot_free_gain > 1.0f ||
        !std::isfinite(cfg.foot_supported_gain) || cfg.foot_supported_gain < 0.0f || cfg.foot_supported_gain > 1.0f) {
        return Status::Error(StatusCode::ValidationError, "temporal_update gains must be finite and in the range 0..1");
    }
    return Status::OK();
}

Status ValidateBodyCalibrationModeConfig(const BodyCalibrationModeConfig& cfg) {
    if (!std::isfinite(cfg.required_seconds) || cfg.required_seconds < 0.5f || cfg.required_seconds > 15.0f ||
        !std::isfinite(cfg.min_overall_confidence) || cfg.min_overall_confidence < 0.0f || cfg.min_overall_confidence > 1.0f ||
        !std::isfinite(cfg.max_segment_cv) || cfg.max_segment_cv < 0.02f || cfg.max_segment_cv > 0.40f) {
        return Status::Error(StatusCode::ValidationError, "body_calibration thresholds must be finite and in valid ranges");
    }
    return Status::OK();
}

} // namespace

ConfigValidationReport ValidateConfig(const AppConfig& cfg) {
    ConfigValidationReport report;

    if (cfg.osc.tracker_space_transform_valid &&
        !TrackerSpaceTransformFinite(
            cfg.osc.tracker_space_position_offset,
            cfg.osc.tracker_space_rotation,
            cfg.osc.tracker_space_scale,
            cfg.osc.tracker_space_role_offsets)) {
        AddConfigIssue(
            report,
            ConfigValidationOutcome::Invalid,
            "osc.tracker_space_*",
            "OSC active tracker-space transform is marked valid but contains invalid numbers, a zero quaternion, non-positive scale, or invalid role offsets");
    }

    if (cfg.osc.manual_tracker_space_transform_valid && !ManualTrackerSpaceFallbackFinite(cfg.osc)) {
        AddConfigIssue(
            report,
            ConfigValidationOutcome::Invalid,
            "osc.manual_tracker_space_*",
            "OSC manual tracker-space fallback is marked valid but contains invalid numbers, a zero quaternion, non-positive scale, or invalid role offsets");
    }

    if (cfg.osc.enabled &&
        !cfg.osc.tracker_space_transform_valid &&
        !ManualTrackerSpaceFallbackFinite(cfg.osc)) {
        AddConfigIssue(
            report,
            ConfigValidationOutcome::Degraded,
            "osc.tracker_space_transform_valid",
            "OSC is enabled but tracker space is not calibrated; config can load, and runtime OSC output stays blocked until a finite active transform or manual fallback exists");
    }

    if (cfg.tracking.monocular.floor_scale_assist_enabled &&
        (!std::isfinite(cfg.tracking.monocular.floor_depth_line_spacing_m) ||
         cfg.tracking.monocular.floor_depth_line_spacing_m <= 0.0f ||
         !std::isfinite(cfg.tracking.monocular.floor_depth_line_spacing_px) ||
         cfg.tracking.monocular.floor_depth_line_spacing_px <= 0.0f)) {
        AddConfigIssue(
            report,
            ConfigValidationOutcome::MissingButDefaultable,
            "tracking.monocular.floor_depth_line_spacing_*",
            "Floor scale assist is enabled without usable spacing; monocular runtime can fall back to body/default/floor-ray depth with lower confidence");
    }

    if (cfg.tracking.monocular.floor_depth_reference_y_px > static_cast<float>(std::max(1, cfg.tracking.monocular.image_height))) {
        AddConfigIssue(
            report,
            ConfigValidationOutcome::Warning,
            "tracking.monocular.floor_depth_reference_y_px",
            "Floor reference Y is outside the current monocular image height; keep the config loadable and let runtime ignore or remap that floor evidence");
    }

    if (cfg.tracking.max_mean_reprojection_error_px != tracking_constants::kReprojectionErrorMaxPx) {
        AddConfigIssue(
            report,
            ConfigValidationOutcome::Warning,
            "tracking.max_mean_reprojection_error_px",
            "Reprojection config differs from tracking_constants::kReprojectionErrorMaxPx; verify body-state quality normalization before treating scores as comparable");
    }

    return report;
}

Result<AppConfig> LoadConfig(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return Status::Error(StatusCode::InvalidArgument, "Config file not readable: " + path.string());
    }

    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        return Status::Error(StatusCode::ValidationError, std::string("Config JSON parse failed: ") + e.what());
    }

    if (const auto schema_status = ValidateRawConfigJsonAgainstSchema(j, path); !schema_status.ok()) {
        return schema_status;
    }

    try {
        AppConfig cfg;
        const auto app = j.value("app", nlohmann::json::object());
        cfg.app.log_file = ReadOr<std::string>(app, "log_file", cfg.app.log_file.string());
        cfg.app.recording_dir = ReadOr<std::string>(app, "recording_dir", cfg.app.recording_dir.string());

        // Legacy web/web_ui sections are intentionally ignored. The browser server was removed;
        // desktop UI is WebView2-only and should not carry a dead local-server config surface.

        const auto tracking = j.value("tracking", nlohmann::json::object());
        const auto tracking_mode = ReadTrackingMode(tracking, cfg.tracking.mode);
        if (!tracking_mode.ok()) {
            return tracking_mode.status();
        }
        cfg.tracking.mode = tracking_mode.value();
        cfg.tracking.model_path = ReadOr<std::string>(tracking, "model_path", cfg.tracking.model_path.string());
        cfg.tracking.depth_postprocess_enabled = ReadOr<bool>(tracking, "depth_postprocess_enabled", cfg.tracking.depth_postprocess_enabled);
        cfg.tracking.depth_postprocess_model_path = ReadOr<std::string>(tracking, "depth_postprocess_model_path", cfg.tracking.depth_postprocess_model_path.string());
        cfg.tracking.depth_postprocess_interval_frames = ReadOr<int>(tracking, "depth_postprocess_interval_frames", cfg.tracking.depth_postprocess_interval_frames);
        cfg.tracking.depth_postprocess_allow_cpu_fallback = ReadOr<bool>(tracking, "depth_postprocess_allow_cpu_fallback", cfg.tracking.depth_postprocess_allow_cpu_fallback);
        cfg.tracking.calibration_path = ReadOr<std::string>(tracking, "calibration_path", cfg.tracking.calibration_path.string());
        // These are intentionally separate knobs. latest_frame_skew_tolerance_ms
        // controls live pair acceptance; max_frame_skew_ms is the broader reported
        // policy/diagnostic ceiling. Legacy configs with only max_frame_skew_ms still
        // seed latest_frame_skew_tolerance_ms for compatibility.
        cfg.tracking.latest_frame_skew_tolerance_ms = ReadOr<double>(
            tracking,
            "latest_frame_skew_tolerance_ms",
            ReadOr<double>(tracking, "max_frame_skew_ms", cfg.tracking.latest_frame_skew_tolerance_ms));
        cfg.tracking.max_frame_skew_ms = ReadOr<double>(
            tracking,
            "max_frame_skew_ms",
            cfg.tracking.latest_frame_skew_tolerance_ms);
        cfg.tracking.stale_frame_timeout_ms = ReadOr<double>(tracking, "stale_frame_timeout_ms", cfg.tracking.stale_frame_timeout_ms);
        cfg.tracking.min_triangulated_seed_count = ReadOr<int>(tracking, "min_triangulated_seed_count", cfg.tracking.min_triangulated_seed_count);
        cfg.tracking.max_mean_reprojection_error_px = ReadOr<double>(tracking, "max_mean_reprojection_error_px", cfg.tracking.max_mean_reprojection_error_px);
        cfg.tracking.stereo_monocular_fallback_enabled = ReadOr<bool>(tracking, "stereo_monocular_fallback_enabled", cfg.tracking.stereo_monocular_fallback_enabled);
        cfg.tracking.use_legacy_solver = ReadOr<bool>(tracking, "use_legacy_solver", cfg.tracking.use_legacy_solver);
        cfg.tracking.enable_replay_recording = ReadOr<bool>(tracking, "enable_replay_recording", cfg.tracking.enable_replay_recording);
        const auto stereo_epipolar = tracking.value("stereo_epipolar", nlohmann::json::object());
        if (const auto s = ReadStereoEpipolarConfig(stereo_epipolar, cfg.tracking.stereo_epipolar); !s.ok()) {
            return s;
        }
        const auto stereo_triangulation = tracking.value("stereo_triangulation", nlohmann::json::object());
        if (const auto s = ReadStereoTriangulationConfig(stereo_triangulation, cfg.tracking.stereo_triangulation); !s.ok()) {
            return s;
        }
        const auto stereo_uncertainty = tracking.value("stereo_uncertainty", nlohmann::json::object());
        if (const auto s = ReadStereoMeasurementUncertaintyConfig(stereo_uncertainty, cfg.tracking.stereo_uncertainty); !s.ok()) {
            return s;
        }
        const auto solver_observation_weighting = tracking.value("solver_observation_weighting", nlohmann::json::object());
        if (const auto s = ReadSolverObservationWeightingConfig(solver_observation_weighting, cfg.tracking.solver_observation_weighting); !s.ok()) {
            return s;
        }
        const auto stereo_identity = tracking.value("stereo_identity", nlohmann::json::object());
        if (const auto s = ReadStereoIdentityEpipolarConfig(stereo_identity, cfg.tracking.stereo_identity); !s.ok()) {
            return s;
        }
        const auto motion_consistency = tracking.value("motion_consistency", nlohmann::json::object());
        if (const auto s = ReadMotionConsistencyConfig(motion_consistency, cfg.tracking.motion_consistency); !s.ok()) {
            return s;
        }
        const auto tracker_ekf = tracking.value("tracker_ekf", nlohmann::json::object());
        if (const auto s = ReadTrackerEkfConfig(tracker_ekf, cfg.tracking.tracker_ekf); !s.ok()) {
            return s;
        }
        const auto temporal_update = tracking.value("temporal_update", nlohmann::json::object());
        if (const auto s = ReadTemporalUpdateConfig(temporal_update, cfg.tracking.temporal_update); !s.ok()) {
            return s;
        }
        const auto body_calibration = tracking.value("body_calibration", nlohmann::json::object());
        if (const auto s = ReadBodyCalibrationModeConfig(body_calibration, cfg.tracking.body_calibration); !s.ok()) {
            return s;
        }
        const auto monocular = tracking.value("monocular", nlohmann::json::object());
        if (const auto s = ReadMonocularTrackingConfig(monocular, cfg.tracking.monocular); !s.ok()) {
            return s;
        }
        const auto stereo_hmd_anchor = tracking.value("stereo_hmd_anchor", nlohmann::json::object());
        cfg.tracking.stereo_hmd_anchor.enabled = ReadOr<bool>(stereo_hmd_anchor, "enabled", cfg.tracking.stereo_hmd_anchor.enabled);
        cfg.tracking.stereo_hmd_anchor.interval_seconds = ReadOr<double>(stereo_hmd_anchor, "interval_seconds", cfg.tracking.stereo_hmd_anchor.interval_seconds);
        cfg.tracking.stereo_hmd_anchor.max_correction_m = ReadOr<float>(stereo_hmd_anchor, "max_correction_m", cfg.tracking.stereo_hmd_anchor.max_correction_m);

        const auto hmd_depth_scale = tracking.value("hmd_depth_scale", nlohmann::json::object());
        cfg.tracking.hmd_depth_scale.enabled = ReadOr<bool>(hmd_depth_scale, "enabled", cfg.tracking.hmd_depth_scale.enabled);
        cfg.tracking.hmd_depth_scale.min_depth_m = ReadOr<float>(hmd_depth_scale, "min_depth_m", cfg.tracking.hmd_depth_scale.min_depth_m);
        cfg.tracking.hmd_depth_scale.max_depth_m = ReadOr<float>(hmd_depth_scale, "max_depth_m", cfg.tracking.hmd_depth_scale.max_depth_m);
        cfg.tracking.hmd_depth_scale.min_scale = ReadOr<float>(hmd_depth_scale, "min_scale", cfg.tracking.hmd_depth_scale.min_scale);
        cfg.tracking.hmd_depth_scale.max_scale = ReadOr<float>(hmd_depth_scale, "max_scale", cfg.tracking.hmd_depth_scale.max_scale);
        cfg.tracking.hmd_depth_scale.max_hold_seconds = ReadOr<double>(hmd_depth_scale, "max_hold_seconds", cfg.tracking.hmd_depth_scale.max_hold_seconds);
        cfg.tracking.hmd_depth_scale.outlier_sigma = ReadOr<float>(hmd_depth_scale, "outlier_sigma", cfg.tracking.hmd_depth_scale.outlier_sigma);
        cfg.tracking.hmd_depth_scale.history_size = ReadOr<int>(hmd_depth_scale, "history_size", cfg.tracking.hmd_depth_scale.history_size);

        const auto anchor_space_mapping = tracking.value("anchor_space_mapping", nlohmann::json::object());
        cfg.tracking.anchor_space_mapping.enabled = ReadOr<bool>(anchor_space_mapping, "enabled", cfg.tracking.anchor_space_mapping.enabled);
        cfg.tracking.anchor_space_mapping.use_hmd = ReadOr<bool>(anchor_space_mapping, "use_hmd", cfg.tracking.anchor_space_mapping.use_hmd);
        cfg.tracking.anchor_space_mapping.use_controllers = ReadOr<bool>(anchor_space_mapping, "use_controllers", cfg.tracking.anchor_space_mapping.use_controllers);
        cfg.tracking.anchor_space_mapping.allow_hmd_only_scale_fallback = ReadOr<bool>(anchor_space_mapping, "allow_hmd_only_scale_fallback", cfg.tracking.anchor_space_mapping.allow_hmd_only_scale_fallback);
        cfg.tracking.anchor_space_mapping.min_anchors_for_pose_refine = ReadOr<int>(anchor_space_mapping, "min_anchors_for_pose_refine", cfg.tracking.anchor_space_mapping.min_anchors_for_pose_refine);
        cfg.tracking.anchor_space_mapping.max_reprojection_error_px = ReadOr<float>(anchor_space_mapping, "max_reprojection_error_px", cfg.tracking.anchor_space_mapping.max_reprojection_error_px);
        cfg.tracking.anchor_space_mapping.target_reprojection_error_px = ReadOr<float>(anchor_space_mapping, "target_reprojection_error_px", cfg.tracking.anchor_space_mapping.target_reprojection_error_px);
        cfg.tracking.anchor_space_mapping.min_depth_scale = ReadOr<float>(anchor_space_mapping, "min_depth_scale", cfg.tracking.anchor_space_mapping.min_depth_scale);
        cfg.tracking.anchor_space_mapping.max_depth_scale = ReadOr<float>(anchor_space_mapping, "max_depth_scale", cfg.tracking.anchor_space_mapping.max_depth_scale);
        cfg.tracking.anchor_space_mapping.room_map_min_update_depth_scale = ReadOr<float>(anchor_space_mapping, "room_map_min_update_depth_scale", cfg.tracking.anchor_space_mapping.room_map_min_update_depth_scale);
        cfg.tracking.anchor_space_mapping.room_map_max_update_depth_scale = ReadOr<float>(anchor_space_mapping, "room_map_max_update_depth_scale", cfg.tracking.anchor_space_mapping.room_map_max_update_depth_scale);
        cfg.tracking.anchor_space_mapping.timestamp_alignment_seconds = ReadOr<double>(anchor_space_mapping, "timestamp_alignment_seconds", cfg.tracking.anchor_space_mapping.timestamp_alignment_seconds);
        cfg.tracking.anchor_space_mapping.log_interval_seconds = ReadOr<double>(anchor_space_mapping, "log_interval_seconds", cfg.tracking.anchor_space_mapping.log_interval_seconds);
        if (const auto v = ReadVec3Or(anchor_space_mapping, "hmd_to_head_keypoint_offset_m", cfg.tracking.anchor_space_mapping.hmd_to_head_keypoint_offset_m); !v.ok()) { return v.status(); } else { cfg.tracking.anchor_space_mapping.hmd_to_head_keypoint_offset_m = v.value(); }
        if (const auto v = ReadVec3Or(anchor_space_mapping, "left_controller_to_wrist_offset_m", cfg.tracking.anchor_space_mapping.left_controller_to_wrist_offset_m); !v.ok()) { return v.status(); } else { cfg.tracking.anchor_space_mapping.left_controller_to_wrist_offset_m = v.value(); }
        if (const auto v = ReadVec3Or(anchor_space_mapping, "right_controller_to_wrist_offset_m", cfg.tracking.anchor_space_mapping.right_controller_to_wrist_offset_m); !v.ok()) { return v.status(); } else { cfg.tracking.anchor_space_mapping.right_controller_to_wrist_offset_m = v.value(); }

        const auto room_depth_map = tracking.value("room_depth_map", nlohmann::json::object());
        cfg.tracking.room_depth_map.enabled = ReadOr<bool>(room_depth_map, "enabled", cfg.tracking.room_depth_map.enabled);
        cfg.tracking.room_depth_map.collect_only = ReadOr<bool>(room_depth_map, "collect_only", cfg.tracking.room_depth_map.collect_only);
        cfg.tracking.room_depth_map.resolution_width = ReadOr<int>(room_depth_map, "resolution_width", cfg.tracking.room_depth_map.resolution_width);
        cfg.tracking.room_depth_map.resolution_height = ReadOr<int>(room_depth_map, "resolution_height", cfg.tracking.room_depth_map.resolution_height);
        cfg.tracking.room_depth_map.min_samples_per_cell = ReadOr<int>(room_depth_map, "min_samples_per_cell", cfg.tracking.room_depth_map.min_samples_per_cell);
        cfg.tracking.room_depth_map.max_cell_variance_m2 = ReadOr<float>(room_depth_map, "max_cell_variance_m2", cfg.tracking.room_depth_map.max_cell_variance_m2);
        cfg.tracking.room_depth_map.min_accepted_frames_before_active = ReadOr<int>(room_depth_map, "min_accepted_frames_before_active", cfg.tracking.room_depth_map.min_accepted_frames_before_active);
        cfg.tracking.room_depth_map.body_mask_dilation_px = ReadOr<int>(room_depth_map, "body_mask_dilation_px", cfg.tracking.room_depth_map.body_mask_dilation_px);
        cfg.tracking.room_depth_map.update_only_when_anchor_quality_good = ReadOr<bool>(room_depth_map, "update_only_when_anchor_quality_good", cfg.tracking.room_depth_map.update_only_when_anchor_quality_good);
        cfg.tracking.room_depth_map.save_path = ReadOr<std::string>(room_depth_map, "save_path", cfg.tracking.room_depth_map.save_path.string());
        cfg.tracking.room_depth_map.load_existing = ReadOr<bool>(room_depth_map, "load_existing", cfg.tracking.room_depth_map.load_existing);
        cfg.tracking.room_depth_map.save_interval_seconds = ReadOr<double>(room_depth_map, "save_interval_seconds", cfg.tracking.room_depth_map.save_interval_seconds);

        const auto stereo_anchor_depth_correction = tracking.value("stereo_anchor_depth_correction", nlohmann::json::object());
        cfg.tracking.stereo_anchor_depth_correction.enabled = ReadOr<bool>(stereo_anchor_depth_correction, "enabled", cfg.tracking.stereo_anchor_depth_correction.enabled);
        cfg.tracking.stereo_anchor_depth_correction.apply_per_frame = ReadOr<bool>(stereo_anchor_depth_correction, "apply_per_frame", cfg.tracking.stereo_anchor_depth_correction.apply_per_frame);
        cfg.tracking.stereo_anchor_depth_correction.min_scale = ReadOr<float>(stereo_anchor_depth_correction, "min_scale", cfg.tracking.stereo_anchor_depth_correction.min_scale);
        cfg.tracking.stereo_anchor_depth_correction.max_scale = ReadOr<float>(stereo_anchor_depth_correction, "max_scale", cfg.tracking.stereo_anchor_depth_correction.max_scale);
        cfg.tracking.stereo_anchor_depth_correction.camera_space_depth_only = ReadOr<bool>(stereo_anchor_depth_correction, "camera_space_depth_only", cfg.tracking.stereo_anchor_depth_correction.camera_space_depth_only);

        const auto inference = j.value("inference", nlohmann::json::object());
        // tracking.model_path is the canonical operator-facing path. inference.model_path
        // is accepted as a legacy fallback only when tracking.model_path is absent.
        if (!tracking.contains("model_path")) {
            cfg.tracking.model_path = ReadOr<std::string>(inference, "model_path", cfg.tracking.model_path.string());
        }
        cfg.inference.model_path = cfg.tracking.model_path;
        cfg.inference.device = ReadOr<std::string>(inference, "device", cfg.inference.device);
        if (cfg.inference.device != "cpu" && cfg.inference.device != "directml" && cfg.inference.device != "directml_strict") {
            return Status::Error(StatusCode::ValidationError, "inference.device must be cpu, directml, or directml_strict");
        }

        // tracking.calibration_path is canonical. The old top-level calibration_path
        // only migrates configs that do not have the tracking-scoped field.
        if (!tracking.contains("calibration_path")) {
            cfg.tracking.calibration_path = ReadOr<std::string>(j, "calibration_path", cfg.tracking.calibration_path.string());
        }
        cfg.calibration_path = cfg.tracking.calibration_path;

        const auto debug = j.value("debug", nlohmann::json::object());
        cfg.debug.replay_log_path = ReadOr<std::string>(debug, "replay_log_path", cfg.debug.replay_log_path.string());
        const auto hmd = j.value("hmd", nlohmann::json::object());
        cfg.hmd.mode = ReadOr<std::string>(hmd, "mode", cfg.hmd.mode);
        cfg.hmd.pose_json_path = ReadOr<std::string>(hmd, "pose_json_path", cfg.hmd.pose_json_path.string());

        const auto osc = j.value("osc", nlohmann::json::object());
        cfg.osc.enabled = ReadOr<bool>(osc, "enabled", cfg.osc.enabled);
        cfg.osc.target_address = ReadOr<std::string>(osc, "target_address", cfg.osc.target_address);
        cfg.osc.target_port = ReadOr<int>(osc, "target_port", cfg.osc.target_port);
        cfg.osc.send_rotations = ReadOr<bool>(osc, "send_rotations", cfg.osc.send_rotations);
        cfg.osc.min_confidence = ReadOr<float>(osc, "min_confidence", cfg.osc.min_confidence);
        cfg.osc.pelvis_tracker_index = ReadOr<int>(osc, "pelvis_tracker_index", cfg.osc.pelvis_tracker_index);
        cfg.osc.left_foot_tracker_index = ReadOr<int>(osc, "left_foot_tracker_index", cfg.osc.left_foot_tracker_index);
        cfg.osc.right_foot_tracker_index = ReadOr<int>(osc, "right_foot_tracker_index", cfg.osc.right_foot_tracker_index);
        cfg.osc.chest_tracker_index = ReadOr<int>(osc, "chest_tracker_index", cfg.osc.chest_tracker_index);
        cfg.osc.left_elbow_tracker_index = ReadOr<int>(osc, "left_elbow_tracker_index", cfg.osc.left_elbow_tracker_index);
        cfg.osc.right_elbow_tracker_index = ReadOr<int>(osc, "right_elbow_tracker_index", cfg.osc.right_elbow_tracker_index);
        cfg.osc.left_knee_tracker_index = ReadOr<int>(osc, "left_knee_tracker_index", cfg.osc.left_knee_tracker_index);
        cfg.osc.right_knee_tracker_index = ReadOr<int>(osc, "right_knee_tracker_index", cfg.osc.right_knee_tracker_index);
        cfg.osc.tracker_space_transform_valid = ReadOr<bool>(osc, "tracker_space_transform_valid", cfg.osc.tracker_space_transform_valid);
        const auto tracker_offset = ReadVec3Or(osc, "tracker_space_position_offset", cfg.osc.tracker_space_position_offset);
        if (!tracker_offset.ok()) {
            return tracker_offset.status();
        }
        cfg.osc.tracker_space_position_offset = tracker_offset.value();
        const auto tracker_rotation = ReadQuatOr(osc, "tracker_space_rotation", cfg.osc.tracker_space_rotation);
        if (!tracker_rotation.ok()) {
            return tracker_rotation.status();
        }
        cfg.osc.tracker_space_rotation = tracker_rotation.value();
        cfg.osc.tracker_space_scale = ReadOr<float>(osc, "tracker_space_scale", cfg.osc.tracker_space_scale);
        const auto role_offsets = ReadTrackerRoleOffsetsOr(osc, "tracker_space_role_offsets", cfg.osc.tracker_space_role_offsets);
        if (!role_offsets.ok()) {
            return role_offsets.status();
        }
        cfg.osc.tracker_space_role_offsets = role_offsets.value();
        cfg.osc.tracker_space_source = ReadOr<std::string>(osc, "tracker_space_source", cfg.osc.tracker_space_source);
        const auto known_tracker_space_source = [](const std::string& source) {
            return source.empty() || source == "none" || source == "manual" || source == "manual_json_file" ||
                source == "steamvr_controller_alignment" || source == "steamvr_controller_alignment_stale" ||
                source == "unavailable";
        };
        if (!known_tracker_space_source(cfg.osc.tracker_space_source)) {
            // Source labels are metadata. A valid numeric transform must not be
            // rejected because its label came from a newer/unknown producer.
            if (!cfg.osc.tracker_space_transform_valid) {
                cfg.osc.tracker_space_source = "none";
            }
        }

        cfg.osc.manual_tracker_space_transform_valid = ReadOr<bool>(osc, "manual_tracker_space_transform_valid", cfg.osc.manual_tracker_space_transform_valid);
        const auto manual_tracker_offset = ReadVec3Or(osc, "manual_tracker_space_position_offset", cfg.osc.manual_tracker_space_position_offset);
        if (!manual_tracker_offset.ok()) {
            return manual_tracker_offset.status();
        }
        cfg.osc.manual_tracker_space_position_offset = manual_tracker_offset.value();
        const auto manual_tracker_rotation = ReadQuatOr(osc, "manual_tracker_space_rotation", cfg.osc.manual_tracker_space_rotation);
        if (!manual_tracker_rotation.ok()) {
            return manual_tracker_rotation.status();
        }
        cfg.osc.manual_tracker_space_rotation = manual_tracker_rotation.value();
        cfg.osc.manual_tracker_space_scale = ReadOr<float>(osc, "manual_tracker_space_scale", cfg.osc.manual_tracker_space_scale);
        const auto manual_role_offsets = ReadTrackerRoleOffsetsOr(osc, "manual_tracker_space_role_offsets", cfg.osc.manual_tracker_space_role_offsets);
        if (!manual_role_offsets.ok()) {
            return manual_role_offsets.status();
        }
        cfg.osc.manual_tracker_space_role_offsets = manual_role_offsets.value();
        cfg.osc.manual_tracker_space_source = ReadOr<std::string>(osc, "manual_tracker_space_source", cfg.osc.manual_tracker_space_source);
        if (!known_tracker_space_source(cfg.osc.manual_tracker_space_source) &&
            !cfg.osc.manual_tracker_space_transform_valid) {
            cfg.osc.manual_tracker_space_source = "none";
        }
        if (!cfg.osc.manual_tracker_space_transform_valid &&
            cfg.osc.tracker_space_transform_valid &&
            (cfg.osc.tracker_space_source == "manual" || cfg.osc.tracker_space_source == "manual_json_file")) {
            cfg.osc.manual_tracker_space_transform_valid = true;
            cfg.osc.manual_tracker_space_position_offset = cfg.osc.tracker_space_position_offset;
            cfg.osc.manual_tracker_space_rotation = cfg.osc.tracker_space_rotation;
            cfg.osc.manual_tracker_space_scale = cfg.osc.tracker_space_scale;
            cfg.osc.manual_tracker_space_role_offsets = cfg.osc.tracker_space_role_offsets;
            cfg.osc.manual_tracker_space_source = cfg.osc.tracker_space_source;
        }

        cfg.osc.steamvr_alignment_status = ReadOr<std::string>(osc, "steamvr_alignment_status", cfg.osc.steamvr_alignment_status);
        cfg.osc.steamvr_alignment_reason = ReadOr<std::string>(osc, "steamvr_alignment_reason", cfg.osc.steamvr_alignment_reason);
        cfg.osc.steamvr_alignment_confidence = ReadOr<float>(osc, "steamvr_alignment_confidence", cfg.osc.steamvr_alignment_confidence);
        cfg.osc.steamvr_alignment_residual_m = ReadOr<float>(osc, "steamvr_alignment_residual_m", cfg.osc.steamvr_alignment_residual_m);
        cfg.osc.steamvr_floor_residual_m = ReadOr<float>(osc, "steamvr_floor_residual_m", cfg.osc.steamvr_floor_residual_m);
        cfg.osc.steamvr_yaw_offset_rad = ReadOr<float>(osc, "steamvr_yaw_offset_rad", cfg.osc.steamvr_yaw_offset_rad);
        cfg.osc.steamvr_scale_ratio = ReadOr<float>(osc, "steamvr_scale_ratio", cfg.osc.steamvr_scale_ratio);
        cfg.osc.steamvr_alignment_body_signature = ReadOr<std::string>(osc, "steamvr_alignment_body_signature", cfg.osc.steamvr_alignment_body_signature);
        cfg.osc.steamvr_alignment_floor_signature = ReadOr<std::string>(osc, "steamvr_alignment_floor_signature", cfg.osc.steamvr_alignment_floor_signature);

        const auto steamvr_tracker_bridge = j.value("steamvr_tracker_bridge", nlohmann::json::object());
        cfg.steamvr_tracker_bridge.enabled = ReadOr<bool>(steamvr_tracker_bridge, "enabled", cfg.steamvr_tracker_bridge.enabled);
        cfg.steamvr_tracker_bridge.target_address = ReadOr<std::string>(steamvr_tracker_bridge, "target_address", cfg.steamvr_tracker_bridge.target_address);
        cfg.steamvr_tracker_bridge.target_port = ReadOr<int>(steamvr_tracker_bridge, "target_port", cfg.steamvr_tracker_bridge.target_port);
        cfg.steamvr_tracker_bridge.min_confidence = ReadOr<float>(steamvr_tracker_bridge, "min_confidence", cfg.steamvr_tracker_bridge.min_confidence);
        cfg.steamvr_tracker_bridge.send_chest = ReadOr<bool>(steamvr_tracker_bridge, "send_chest", cfg.steamvr_tracker_bridge.send_chest);
        cfg.steamvr_tracker_bridge.send_elbows = ReadOr<bool>(steamvr_tracker_bridge, "send_elbows", cfg.steamvr_tracker_bridge.send_elbows);
        cfg.steamvr_tracker_bridge.send_knees = ReadOr<bool>(steamvr_tracker_bridge, "send_knees", cfg.steamvr_tracker_bridge.send_knees);

        const auto cam_a = j.value("camera_a", nlohmann::json::object());
        if (const auto s = ReadCameraConfig(cam_a, cfg.camera_a); !s.ok()) {
            return s;
        }

        const auto cam_b = j.value("camera_b", nlohmann::json::object());
        if (const auto s = ReadCameraConfig(cam_b, cfg.camera_b); !s.ok()) {
            return s;
        }
        if (!monocular.contains("image_width")) {
            cfg.tracking.monocular.image_width = cfg.camera_a.width;
        }
        if (!monocular.contains("image_height")) {
            cfg.tracking.monocular.image_height = cfg.camera_a.height;
        }

        if (cfg.hmd.mode != "null" && cfg.hmd.mode != "json_file" && cfg.hmd.mode != "steamvr") {
            return Status::Error(StatusCode::ValidationError, "HMD mode must be null, json_file, or steamvr");
        }
        if (!std::isfinite(cfg.tracking.stereo_hmd_anchor.interval_seconds) || cfg.tracking.stereo_hmd_anchor.interval_seconds <= 0.0 ||
            !std::isfinite(cfg.tracking.stereo_hmd_anchor.max_correction_m) || cfg.tracking.stereo_hmd_anchor.max_correction_m <= 0.0f) {
            return Status::Error(StatusCode::ValidationError, "tracking.stereo_hmd_anchor contains invalid bounds");
        }
        if (!std::isfinite(cfg.tracking.hmd_depth_scale.min_depth_m) || !std::isfinite(cfg.tracking.hmd_depth_scale.max_depth_m) ||
            cfg.tracking.hmd_depth_scale.min_depth_m <= 0.0f || cfg.tracking.hmd_depth_scale.max_depth_m <= cfg.tracking.hmd_depth_scale.min_depth_m ||
            !std::isfinite(cfg.tracking.hmd_depth_scale.min_scale) || !std::isfinite(cfg.tracking.hmd_depth_scale.max_scale) ||
            cfg.tracking.hmd_depth_scale.min_scale <= 0.0f || cfg.tracking.hmd_depth_scale.max_scale < cfg.tracking.hmd_depth_scale.min_scale ||
            !std::isfinite(cfg.tracking.hmd_depth_scale.max_hold_seconds) || cfg.tracking.hmd_depth_scale.max_hold_seconds < 0.0 ||
            !std::isfinite(cfg.tracking.hmd_depth_scale.outlier_sigma) || cfg.tracking.hmd_depth_scale.outlier_sigma < 1.0f ||
            cfg.tracking.hmd_depth_scale.history_size < 1 || cfg.tracking.hmd_depth_scale.history_size > 15) {
            return Status::Error(StatusCode::ValidationError, "tracking.hmd_depth_scale contains invalid bounds");
        }
        if (!std::isfinite(cfg.tracking.anchor_space_mapping.min_depth_scale) || !std::isfinite(cfg.tracking.anchor_space_mapping.max_depth_scale) ||
            cfg.tracking.anchor_space_mapping.min_depth_scale <= 0.0f || cfg.tracking.anchor_space_mapping.max_depth_scale < cfg.tracking.anchor_space_mapping.min_depth_scale ||
            !std::isfinite(cfg.tracking.anchor_space_mapping.max_reprojection_error_px) || cfg.tracking.anchor_space_mapping.max_reprojection_error_px <= 0.0f ||
            !std::isfinite(cfg.tracking.anchor_space_mapping.target_reprojection_error_px) || cfg.tracking.anchor_space_mapping.target_reprojection_error_px <= 0.0f ||
            cfg.tracking.anchor_space_mapping.min_anchors_for_pose_refine < 1 || cfg.tracking.anchor_space_mapping.min_anchors_for_pose_refine > 3 ||
            !std::isfinite(cfg.tracking.anchor_space_mapping.timestamp_alignment_seconds) || cfg.tracking.anchor_space_mapping.timestamp_alignment_seconds < 0.0 ||
            !std::isfinite(cfg.tracking.anchor_space_mapping.log_interval_seconds) || cfg.tracking.anchor_space_mapping.log_interval_seconds <= 0.0) {
            return Status::Error(StatusCode::ValidationError, "tracking.anchor_space_mapping contains invalid bounds");
        }
        if (cfg.tracking.room_depth_map.resolution_width <= 0 || cfg.tracking.room_depth_map.resolution_height <= 0 ||
            cfg.tracking.room_depth_map.min_samples_per_cell < 1 || cfg.tracking.room_depth_map.min_accepted_frames_before_active < 1 ||
            cfg.tracking.room_depth_map.body_mask_dilation_px < 0 ||
            !std::isfinite(cfg.tracking.room_depth_map.max_cell_variance_m2) || cfg.tracking.room_depth_map.max_cell_variance_m2 <= 0.0f ||
            !std::isfinite(cfg.tracking.room_depth_map.save_interval_seconds) || cfg.tracking.room_depth_map.save_interval_seconds <= 0.0) {
            return Status::Error(StatusCode::ValidationError, "tracking.room_depth_map contains invalid bounds");
        }
        if (!std::isfinite(cfg.tracking.stereo_anchor_depth_correction.min_scale) || !std::isfinite(cfg.tracking.stereo_anchor_depth_correction.max_scale) ||
            cfg.tracking.stereo_anchor_depth_correction.min_scale <= 0.0f || cfg.tracking.stereo_anchor_depth_correction.max_scale < cfg.tracking.stereo_anchor_depth_correction.min_scale) {
            return Status::Error(StatusCode::ValidationError, "tracking.stereo_anchor_depth_correction contains invalid bounds");
        }
        if (cfg.osc.target_port <= 0 || cfg.osc.target_port > 65535) {
            return Status::Error(StatusCode::ValidationError, "Config contains invalid OSC port");
        }
        if (cfg.osc.min_confidence < 0.0f || cfg.osc.min_confidence > 1.0f) {
            return Status::Error(StatusCode::ValidationError, "OSC min_confidence must be in the range 0..1");
        }
        if (cfg.steamvr_tracker_bridge.target_port <= 0 || cfg.steamvr_tracker_bridge.target_port > 65535) {
            return Status::Error(StatusCode::ValidationError, "SteamVR tracker bridge contains invalid target port");
        }
        if (cfg.steamvr_tracker_bridge.min_confidence < 0.0f || cfg.steamvr_tracker_bridge.min_confidence > 1.0f) {
            return Status::Error(StatusCode::ValidationError, "SteamVR tracker bridge min_confidence must be in the range 0..1");
        }
        const bool active_transform_numbers_valid = TrackerSpaceTransformFinite(
            cfg.osc.tracker_space_position_offset,
            cfg.osc.tracker_space_rotation,
            cfg.osc.tracker_space_scale,
            cfg.osc.tracker_space_role_offsets);
        const bool manual_fallback_valid = ManualTrackerSpaceFallbackFinite(cfg.osc);
        if (cfg.osc.tracker_space_transform_valid && !active_transform_numbers_valid) {
            return Status::Error(StatusCode::ValidationError, "OSC active tracker-space transform is marked valid but contains invalid numbers, a zero quaternion, non-positive scale, or invalid role offsets");
        }
        if (cfg.osc.manual_tracker_space_transform_valid && !manual_fallback_valid) {
            return Status::Error(StatusCode::ValidationError, "OSC manual tracker-space fallback is marked valid but contains invalid numbers, a zero quaternion, non-positive scale, or invalid role offsets");
        }
        if (!std::isfinite(cfg.osc.tracker_space_scale) || cfg.osc.tracker_space_scale <= 0.0f) {
            return Status::Error(StatusCode::ValidationError, "OSC tracker_space_scale must be a positive finite number");
        }
        if (!std::isfinite(cfg.osc.manual_tracker_space_scale) || cfg.osc.manual_tracker_space_scale <= 0.0f) {
            return Status::Error(StatusCode::ValidationError, "OSC manual_tracker_space_scale must be a positive finite number");
        }
        if (!std::isfinite(cfg.osc.steamvr_alignment_confidence) ||
            cfg.osc.steamvr_alignment_confidence < 0.0f || cfg.osc.steamvr_alignment_confidence > 1.0f ||
            !std::isfinite(cfg.osc.steamvr_alignment_residual_m) ||
            !std::isfinite(cfg.osc.steamvr_floor_residual_m) ||
            !std::isfinite(cfg.osc.steamvr_yaw_offset_rad) ||
            !std::isfinite(cfg.osc.steamvr_scale_ratio) ||
            cfg.osc.steamvr_scale_ratio <= 0.0f) {
            return Status::Error(StatusCode::ValidationError, "SteamVR alignment metrics must be finite and confidence must be in 0..1");
        }
        const auto valid_required_tracker_index = [](int v) { return v >= 1 && v <= 8; };
        const auto valid_optional_tracker_index = [](int v) { return v >= 0 && v <= 8; };
        if (!valid_required_tracker_index(cfg.osc.pelvis_tracker_index) ||
            !valid_required_tracker_index(cfg.osc.left_foot_tracker_index) ||
            !valid_required_tracker_index(cfg.osc.right_foot_tracker_index) ||
            !valid_optional_tracker_index(cfg.osc.chest_tracker_index) ||
            !valid_optional_tracker_index(cfg.osc.left_elbow_tracker_index) ||
            !valid_optional_tracker_index(cfg.osc.right_elbow_tracker_index) ||
            !valid_optional_tracker_index(cfg.osc.left_knee_tracker_index) ||
            !valid_optional_tracker_index(cfg.osc.right_knee_tracker_index)) {
            return Status::Error(StatusCode::ValidationError, "OSC pelvis and foot tracker indices must be in the range 1..8; upper-body and knee tracker indices may be 0..8");
        }
        const int tracker_indices[] = {
            cfg.osc.pelvis_tracker_index,
            cfg.osc.left_foot_tracker_index,
            cfg.osc.right_foot_tracker_index,
            cfg.osc.chest_tracker_index,
            cfg.osc.left_elbow_tracker_index,
            cfg.osc.right_elbow_tracker_index,
            cfg.osc.left_knee_tracker_index,
            cfg.osc.right_knee_tracker_index
        };
        for (std::size_t i = 0; i < (sizeof(tracker_indices) / sizeof(tracker_indices[0])); ++i) {
            if (tracker_indices[i] == 0) {
                continue;
            }
            for (std::size_t k = i + 1; k < (sizeof(tracker_indices) / sizeof(tracker_indices[0])); ++k) {
                if (tracker_indices[k] != 0 && tracker_indices[i] == tracker_indices[k]) {
                    return Status::Error(StatusCode::ValidationError, "OSC tracker indices must be unique");
                }
            }
        }
        if (!std::isfinite(cfg.tracking.latest_frame_skew_tolerance_ms) || cfg.tracking.latest_frame_skew_tolerance_ms <= 0.0 ||
            !std::isfinite(cfg.tracking.max_frame_skew_ms) || cfg.tracking.max_frame_skew_ms <= 0.0 ||
            !std::isfinite(cfg.tracking.stale_frame_timeout_ms) || cfg.tracking.stale_frame_timeout_ms <= 0.0 ||
            !std::isfinite(cfg.tracking.max_mean_reprojection_error_px) || cfg.tracking.max_mean_reprojection_error_px <= 0.0) {
            return Status::Error(StatusCode::ValidationError, "Tracking timing and reprojection thresholds must be positive finite numbers");
        }
        if (cfg.tracking.depth_postprocess_interval_frames < 1) {
            return Status::Error(StatusCode::ValidationError, "tracking.depth_postprocess_interval_frames must be >= 1");
        }
        if (const auto s = ValidateStereoEpipolarConfig(cfg.tracking.stereo_epipolar); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateStereoTriangulationConfig(cfg.tracking.stereo_triangulation); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateStereoMeasurementUncertaintyConfig(cfg.tracking.stereo_uncertainty); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateSolverObservationWeightingConfig(cfg.tracking.solver_observation_weighting); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateStereoIdentityEpipolarConfig(cfg.tracking.stereo_identity); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateMotionConsistencyConfig(cfg.tracking.motion_consistency); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateTrackerEkfConfig(cfg.tracking.tracker_ekf); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateTemporalUpdateConfig(cfg.tracking.temporal_update); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateBodyCalibrationModeConfig(cfg.tracking.body_calibration); !s.ok()) {
            return s;
        }
        if (const auto s = ValidateMonocularTrackingConfig(cfg.tracking.monocular); !s.ok()) {
            return s;
        }
        if (cfg.tracking.min_triangulated_seed_count < 1 || cfg.tracking.min_triangulated_seed_count > static_cast<int>(kHalpe26Count)) {
            return Status::Error(StatusCode::ValidationError, "min_triangulated_seed_count must be between 1 and 26");
        }
        const bool stereo_mode = cfg.tracking.mode == TrackingMode::Stereo;
        const auto validate_camera_source = [](const CameraConfig& c, const char* label) -> Status {
            if (c.source != "opencv" && c.source != "network_mjpeg") {
                return Status::Error(StatusCode::ValidationError, std::string(label) + " source must be opencv or network_mjpeg");
            }
            if (c.source == "opencv" && c.device_index < 0) {
                return Status::Error(StatusCode::ValidationError, std::string(label) + " opencv device_index must be non-negative");
            }
            if (c.source == "network_mjpeg") {
                if (c.network_port <= 0 || c.network_port > 65535) {
                    return Status::Error(StatusCode::ValidationError, std::string(label) + " network_port must be in 1..65535");
                }
                if (c.network_read_timeout_ms < 50 || c.network_read_timeout_ms > 60000) {
                    return Status::Error(StatusCode::ValidationError, std::string(label) + " network_read_timeout_ms must be in 50..60000");
                }
                if (c.network_max_frame_bytes < 4096 || c.network_max_frame_bytes > 64 * 1024 * 1024) {
                    return Status::Error(StatusCode::ValidationError, std::string(label) + " network_max_frame_bytes must be in 4096..67108864");
                }
            }
            if (c.width <= 0 || c.height <= 0 || c.fps <= 0) {
                return Status::Error(StatusCode::ValidationError, std::string(label) + " dimensions and FPS must be positive");
            }
            return Status::OK();
        };
        if (const auto s = validate_camera_source(cfg.camera_a, "camera_a"); !s.ok()) {
            return s;
        }
        if (stereo_mode) {
            if (const auto s = validate_camera_source(cfg.camera_b, "camera_b"); !s.ok()) {
                return s;
            }
        }
        if (stereo_mode &&
            cfg.camera_a.source == "opencv" && cfg.camera_b.source == "opencv" &&
            cfg.camera_a.device_index == cfg.camera_b.device_index) {
            return Status::Error(StatusCode::ValidationError, "camera_a and camera_b must use different OpenCV device_index values in stereo mode");
        }
        if (stereo_mode &&
            cfg.camera_a.source == "network_mjpeg" && cfg.camera_b.source == "network_mjpeg" &&
            cfg.camera_a.network_bind_address == cfg.camera_b.network_bind_address &&
            cfg.camera_a.network_port == cfg.camera_b.network_port) {
            return Status::Error(StatusCode::ValidationError, "camera_a and camera_b network_mjpeg sources must not listen on the same address and port");
        }
        const auto validate_initial_roi = [](const CameraConfig& c, const char* label) -> Status {
            if (!c.initial_roi_enabled) {
                return Status::OK();
            }
            if (c.initial_roi.width <= 0.0f || c.initial_roi.height <= 0.0f) {
                return Status::Error(StatusCode::ValidationError, std::string(label) + " initial ROI width/height must be positive");
            }
            if (c.initial_roi.x < 0.0f || c.initial_roi.y < 0.0f) {
                return Status::Error(StatusCode::ValidationError, std::string(label) + " initial ROI x/y must be non-negative");
            }
            if (c.initial_roi_normalized &&
                (c.initial_roi.x + c.initial_roi.width > 1.0f ||
                 c.initial_roi.y + c.initial_roi.height > 1.0f)) {
                return Status::Error(StatusCode::ValidationError, std::string(label) + " normalized initial ROI must stay inside 0..1");
            }
            return Status::OK();
        };
        if (const auto s = validate_initial_roi(cfg.camera_a, "camera_a"); !s.ok()) {
            return s;
        }
        if (stereo_mode) {
            if (const auto s = validate_initial_roi(cfg.camera_b, "camera_b"); !s.ok()) {
                return s;
            }
        }

        const ConfigValidationReport validation = ValidateConfig(cfg);
        for (const auto& issue : validation.issues) {
            if (issue.outcome == ConfigValidationOutcome::Invalid) {
                return Status::Error(StatusCode::ValidationError, issue.message);
            }
        }

        return cfg;
    } catch (const std::exception& e) {
        return Status::Error(StatusCode::ValidationError, std::string("Config JSON validation failed: ") + e.what());
    }
}

Status SaveDefaultConfig(const std::filesystem::path& path) {
    static constexpr const char* kDefaultConfig = R"JSON(
{
  "$schema": "bodytracker-config.schema.json",
  "app": {
    "log_file": "bodytracker.log",
    "recording_dir": "recordings"
  },
  "camera_a": {
    "device_index": 0,
    "fps": 30,
    "height": 720,
    "initial_roi": [
      0,
      0,
      1,
      1
    ],
    "initial_roi_enabled": false,
    "initial_roi_normalized": true,
    "network_bind_address": "0.0.0.0",
    "network_max_frame_bytes": 8388608,
    "network_port": 39555,
    "network_read_timeout_ms": 1000,
    "source": "network_mjpeg",
    "width": 1280
  },
  "camera_b": {
    "device_index": 1,
    "fps": 60,
    "height": 720,
    "initial_roi": [
      0.0,
      0.0,
      1.0,
      1.0
    ],
    "initial_roi_enabled": false,
    "initial_roi_normalized": true,
    "network_bind_address": "0.0.0.0",
    "network_max_frame_bytes": 8388608,
    "network_port": 39556,
    "network_read_timeout_ms": 1000,
    "source": "opencv",
    "width": 1280
  },
  "debug": {
    "replay_log_path": ""
  },
  "hmd": {
    "mode": "null",
    "pose_json_path": "hmd_pose.json"
  },
  "inference": {
    "device": "directml"
  },
  "osc": {
    "_comment_min_confidence": "See src/tracking/tracking_constants.h::kVisibleConfidenceThreshold.",
    "chest_tracker_index": 4,
    "enabled": false,
    "left_elbow_tracker_index": 5,
    "left_foot_tracker_index": 2,
    "left_knee_tracker_index": 0,
    "manual_tracker_space_position_offset": [
      0.0,
      0.0,
      0.0
    ],
    "manual_tracker_space_role_offsets": [
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ]
    ],
    "manual_tracker_space_rotation": [
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "manual_tracker_space_scale": 1.0,
    "manual_tracker_space_source": "manual",
    "manual_tracker_space_transform_valid": false,
    "min_confidence": 0.20000000298023224,
    "pelvis_tracker_index": 1,
    "right_elbow_tracker_index": 6,
    "right_foot_tracker_index": 3,
    "right_knee_tracker_index": 0,
    "send_rotations": true,
    "steamvr_alignment_body_signature": "",
    "steamvr_alignment_confidence": 0.0,
    "steamvr_alignment_floor_signature": "",
    "steamvr_alignment_reason": "",
    "steamvr_alignment_residual_m": 0.0,
    "steamvr_alignment_status": "idle",
    "steamvr_floor_residual_m": 0.0,
    "steamvr_scale_ratio": 1.0,
    "steamvr_yaw_offset_rad": 0.0,
    "target_address": "127.0.0.1",
    "target_port": 9000,
    "tracker_space_position_offset": [
      0.0,
      0.0,
      0.0
    ],
    "tracker_space_role_offsets": [
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ],
      [
        0.0,
        0.0,
        0.0
      ]
    ],
    "tracker_space_rotation": [
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "tracker_space_scale": 1.0,
    "tracker_space_source": "manual",
    "tracker_space_transform_valid": false
  },
  "tracking": {
    "_comment_reprojection": "See src/tracking/tracking_constants.h::kReprojectionErrorMaxPx.",
    "body_calibration": {
      "_comment_min_overall_confidence": "See src/tracking/tracking_constants.h::kIdentityStableThreshold.",
      "auto_persist": true,
      "enabled": false,
      "max_segment_cv": 0.11999999731779099,
      "min_overall_confidence": 0.550000011920929,
      "required_seconds": 2.5
    },
    "calibration_path": "calib/default.json",
    "depth_postprocess_allow_cpu_fallback": false,
    "depth_postprocess_enabled": false,
    "depth_postprocess_interval_frames": 4,
    "depth_postprocess_model_path": "models/rtmw3d-x-cocktail14-384x288.onnx",
    "enable_replay_recording": false,
    "latest_frame_skew_tolerance_ms": 18.0,
    "max_frame_skew_ms": 18.0,
    "max_mean_reprojection_error_px": 45.0,
    "min_triangulated_seed_count": 3,
    "mode": "monocular",
    "model_path": "models/rtmw-dw-x-l-cocktail14-384x288.onnx",
    "monocular": {
      "_comment_bounds": "FOV/depth bounds are documented in src/tracking/tracking_constants.h.",
      "camera_height_m": 1.2000000476837158,
      "default_depth_m": 2.200000047683716,
      "depth_confidence_scale": 0.550000011920929,
      "floor_camera_orientation_confidence": 0.0,
      "floor_camera_orientation_enabled": false,
      "floor_camera_pitch_rad": 0.0,
      "floor_camera_roll_rad": 0.0,
      "floor_depth_confidence": 0.6499999761581421,
      "floor_depth_line_spacing_m": 0.0,
      "floor_depth_line_spacing_px": 0.0,
      "floor_depth_reference_m": 0.0,
      "floor_depth_reference_y_px": 0.0,
      "floor_distortion_confidence": 0.0,
      "floor_distortion_correction_enabled": false,
      "floor_from_image": [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0
      ],
      "floor_geometry_calibration_enabled": false,
      "floor_geometry_confidence": 0.0,
      "floor_geometry_type": "unknown",
      "floor_projective_confidence": 0.0,
      "floor_projective_homography_enabled": false,
      "floor_radial_k1": 0.0,
      "floor_radial_k2": 0.0,
      "floor_scale_assist_enabled": false,
      "floor_second_axis_spacing_m": 0.0,
      "floor_tangential_p1": 0.0,
      "floor_tangential_p2": 0.0,
      "horizontal_fov_deg": 70.0,
      "image_from_floor": [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0
      ],
      "image_height": 720,
      "image_width": 1280,
      "min_keypoint_confidence": 0.05000000074505806,
      "min_seed_count": 4,
      "user_height_m": 1.7000000476837158
    },
    "motion_consistency": {
      "confirm_frames": 2,
      "confirm_frames_max": 6,
      "confirm_scale_m": 0.04,
      "contact_root_correction_gain": 0.20000000298023224,
      "contact_root_max_correction_m": 0.014999999664723873,
      "contact_root_max_disagreement_m": 0.012000000104308128,
      "contact_root_max_residual_m": 0.03500000014901161,
      "contact_root_min_alignment": 0.75,
      "contact_root_min_support_confidence": 0.75,
      "enabled": true,
      "foot_max_accel_mps2": 60.0,
      "foot_max_speed_mps": 7.0,
      "max_direction_deviation_deg": 45.0,
      "max_lateral_deviation_ratio": 0.6499999761581421,
      "max_speed_change_ratio": 2.5,
      "min_motion_m": 0.014999999664723873,
      "one_euro_beta": 0.017999999225139618,
      "one_euro_d_cutoff_hz": 1.0,
      "one_euro_enabled": true,
      "one_euro_min_cutoff_hz": 1.2000000476837158,
      "planted_foot_max_drift_m": 0.05000000074505806,
      "planted_foot_release_confirm_frames": 2,
      "reject_confidence_decay_per_second": 1.2000000476837158,
      "root_max_accel_mps2": 22.0,
      "root_max_speed_mps": 3.5,
      "stationary_deadzone_m": 0.006000000052154064
    },
    "solver_observation_weighting": {
      "fallback_stddev_m": 1.0,
      "max_temporal_process_stddev_m": 1.0,
      "max_weight_scale": 10.0,
      "min_stddev_for_weight_m": 0.015,
      "reference_stddev_m": 0.05,
      "temporal_process_variance_m2_per_s": 0.1225
    },
    "stale_frame_timeout_ms": 250.0,
    "stereo_epipolar": {
      "enabled": true,
      "hard_mismatch_rejects_degraded_pair": false,
      "hard_mismatch_rejects_fresh_pair": true,
      "hard_threshold_px": 18.0,
      "min_confidence_floor": 0.1,
      "soft_threshold_px": 2.5
    },
    "stereo_identity": {
      "hard_threshold_px": 18.0,
      "identity_mahalanobis_score_scale": 25.0,
      "identity_max_mahalanobis_sq": 25.0,
      "identity_prior_depth_stddev_m": 2.5,
      "identity_prior_lateral_stddev_m": 0.35,
      "identity_swap_nll_margin": 9.0,
      "min_assignment_score": 0.45,
      "min_detection_support": 0.15,
      "min_scored_lateral_pairs": 2,
      "partial_coverage_swap_margin": 0.1,
      "soft_threshold_px": 2.5,
      "strong_consistency_guard": 0.55,
      "swap_absolute_margin": 0.18,
      "swap_ratio_margin": 1.35,
      "uncertainty_swap_margin_scale": 0.2
    },
    "stereo_monocular_fallback_enabled": true,
    "stereo_triangulation": {
      "max_dlt_condition_number": 1000.0,
      "max_ray_closest_distance_m": 0.18,
      "min_dlt_strength_ratio": 0.001,
      "min_ray_angle_deg": 0.2,
      "min_single_camera_quality": 0.06,
      "min_stereo_reprojection_confidence": 0.055,
      "single_camera_depth_confidence_scale": 0.58
    },
    "stereo_uncertainty": {
      "max_conditioning_scale": 10.0,
      "max_reported_position_stddev_m": 100.0,
      "min_depth_stddev_m": 0.005,
      "min_image_noise_sigma_px": 0.35,
      "min_lateral_stddev_m": 0.002
    },
    "temporal_update": {
      "foot_free_gain": 0.55,
      "foot_supported_gain": 0.12,
      "free_gain": 0.75,
      "supported_gain": 0.2
    },
    "tracker_ekf": {
      "enabled": true,
      "foot_orientation_gain": 0.35,
      "mahalanobis_gate_chi2": 16.27,
      "mahalanobis_gate_enabled": true,
      "max_measurement_variance_m2": 0.01,
      "min_measurement_variance_m2": 2.5e-05,
      "missing_velocity_decay": 0.92,
      "outlier_variance_scale": 64.0,
      "process_noise_mps2": 8.0,
      "support_variance_scale": 0.25
    },
    "use_legacy_solver": false,
    "hmd_depth_scale": {
      "enabled": false,
      "min_depth_m": 0.3,
      "max_depth_m": 8.0,
      "min_scale": 0.33,
      "max_scale": 3.0,
      "max_hold_seconds": 0.25,
      "outlier_sigma": 4.0,
      "history_size": 15
    },
    "stereo_hmd_anchor": {
      "enabled": false,
      "interval_seconds": 5.0,
      "max_correction_m": 1.25
    },
    "anchor_space_mapping": {
      "enabled": true,
      "use_hmd": true,
      "use_controllers": true,
      "allow_hmd_only_scale_fallback": true,
      "min_anchors_for_pose_refine": 3,
      "max_reprojection_error_px": 20.0,
      "target_reprojection_error_px": 8.0,
      "min_depth_scale": 0.75,
      "max_depth_scale": 1.35,
      "room_map_min_update_depth_scale": 0.75,
      "room_map_max_update_depth_scale": 1.35,
      "timestamp_alignment_seconds": 0.05,
      "log_interval_seconds": 1.0,
      "hmd_to_head_keypoint_offset_m": [0.0, 0.0, 0.0],
      "left_controller_to_wrist_offset_m": [0.0, 0.0, 0.0],
      "right_controller_to_wrist_offset_m": [0.0, 0.0, 0.0]
    },
    "room_depth_map": {
      "enabled": true,
      "collect_only": true,
      "resolution_width": 320,
      "resolution_height": 180,
      "min_samples_per_cell": 8,
      "max_cell_variance_m2": 0.01,
      "min_accepted_frames_before_active": 1000,
      "body_mask_dilation_px": 24,
      "update_only_when_anchor_quality_good": true,
      "save_path": "data/room_depth_map.bin",
      "load_existing": true,
      "save_interval_seconds": 30.0
    },
    "stereo_anchor_depth_correction": {
      "enabled": true,
      "apply_per_frame": true,
      "min_scale": 0.90,
      "max_scale": 1.10,
      "camera_space_depth_only": true
    }
  },
  "steamvr_tracker_bridge": {
    "enabled": true,
    "target_address": "127.0.0.1",
    "target_port": 39560,
    "min_confidence": 0.2,
    "send_chest": true,
    "send_elbows": true,
    "send_knees": false
  }
}
)JSON";
    return WriteTextFileAtomically(path, std::string(kDefaultConfig) + '\n', "default config");
}


} // namespace bt
