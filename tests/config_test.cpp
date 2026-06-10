#include "core/config.h"
#include "test_check.h"

#include <filesystem>
#include <fstream>

namespace {

std::filesystem::path TempConfigPath(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

void WriteFile(const std::filesystem::path& path, const char* text) {
    std::ofstream out(path);
    out << text;
}

} // namespace

int main() {
    const auto valid_path = TempConfigPath("bodytracker_valid_config_test.json");
    WriteFile(valid_path, R"JSON({
  "web": {"enabled": true, "bind_address": "127.0.0.1", "port": 8088},
  "tracking": {
    "max_frame_skew_ms": 22.5,
    "motion_consistency": {
      "contact_root_correction_gain": 0.33,
      "contact_root_max_correction_m": 0.021,
      "contact_root_max_residual_m": 0.044,
      "contact_root_max_disagreement_m": 0.013,
      "contact_root_min_alignment": 0.66,
      "contact_root_min_support_confidence": 0.81
    },
    "tracker_ekf": {
      "enabled": false,
      "process_noise_mps2": 3.5,
      "min_measurement_variance_m2": 0.0002,
      "max_measurement_variance_m2": 0.02,
      "support_variance_scale": 0.5,
      "missing_velocity_decay": 0.75,
      "foot_orientation_gain": 0.40
    },
    "temporal_update": {
      "free_gain": 0.60,
      "supported_gain": 0.15,
      "foot_free_gain": 0.50,
      "foot_supported_gain": 0.10
    },
    "body_calibration": {
      "enabled": true,
      "auto_persist": false,
      "required_seconds": 3.0,
      "min_overall_confidence": 0.70,
      "max_segment_cv": 0.10
    }
  }
})JSON");

    const auto loaded = bt::LoadConfig(valid_path);
    BT_CHECK(loaded.ok());
    // Legacy web/web_ui config is ignored; the local browser server was removed.
    BT_CHECK_NEAR(loaded.value().tracking.latest_frame_skew_tolerance_ms, 22.5, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.max_frame_skew_ms, 22.5, 1e-6);
    BT_CHECK(!loaded.value().tracking.tracker_ekf.enabled);
    BT_CHECK_NEAR(loaded.value().tracking.tracker_ekf.process_noise_mps2, 3.5, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.tracker_ekf.min_measurement_variance_m2, 0.0002, 1e-8);
    BT_CHECK_NEAR(loaded.value().tracking.tracker_ekf.max_measurement_variance_m2, 0.02, 1e-8);
    BT_CHECK_NEAR(loaded.value().tracking.tracker_ekf.support_variance_scale, 0.5, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.tracker_ekf.missing_velocity_decay, 0.75, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.tracker_ekf.foot_orientation_gain, 0.40, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.temporal_update.free_gain, 0.60, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.temporal_update.supported_gain, 0.15, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.temporal_update.foot_free_gain, 0.50, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.temporal_update.foot_supported_gain, 0.10, 1e-6);
    BT_CHECK(loaded.value().tracking.body_calibration.enabled);
    BT_CHECK(!loaded.value().tracking.body_calibration.auto_persist);
    BT_CHECK_NEAR(loaded.value().tracking.body_calibration.required_seconds, 3.0, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.body_calibration.min_overall_confidence, 0.70, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.body_calibration.max_segment_cv, 0.10, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.motion_consistency.contact_root_correction_gain, 0.33, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.motion_consistency.contact_root_max_correction_m, 0.021, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.motion_consistency.contact_root_max_residual_m, 0.044, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.motion_consistency.contact_root_max_disagreement_m, 0.013, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.motion_consistency.contact_root_min_alignment, 0.66, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.motion_consistency.contact_root_min_support_confidence, 0.81, 1e-6);
    BT_CHECK(loaded.value().tracking.mode == bt::TrackingMode::Stereo);
    BT_CHECK(loaded.value().hmd.mode == "null");
    BT_CHECK(!loaded.value().tracking.hmd_depth_scale.enabled);
    BT_CHECK(!loaded.value().tracking.stereo_hmd_anchor.enabled);
    BT_CHECK(loaded.value().tracking.anchor_space_mapping.enabled);
    BT_CHECK(loaded.value().tracking.anchor_space_mapping.use_hmd);
    BT_CHECK(loaded.value().tracking.anchor_space_mapping.use_controllers);
    BT_CHECK_NEAR(loaded.value().tracking.anchor_space_mapping.min_depth_scale, 0.75, 1e-6);
    BT_CHECK_NEAR(loaded.value().tracking.anchor_space_mapping.max_depth_scale, 1.35, 1e-6);
    BT_CHECK(loaded.value().tracking.room_depth_map.enabled);
    BT_CHECK(loaded.value().tracking.room_depth_map.collect_only);
    BT_CHECK(loaded.value().tracking.room_depth_map.resolution_width == 320);
    BT_CHECK(loaded.value().tracking.stereo_anchor_depth_correction.enabled);
    BT_CHECK(loaded.value().tracking.stereo_anchor_depth_correction.camera_space_depth_only);

    const auto runtime_math_path = TempConfigPath("bodytracker_runtime_math_config_test.json");
    WriteFile(runtime_math_path, R"JSON({
  "tracking": {
    "solver_observation_weighting": {
      "reference_stddev_m": 0.06,
      "min_stddev_for_weight_m": 0.02,
      "max_weight_scale": 8.0,
      "fallback_stddev_m": 1.20,
      "temporal_process_variance_m2_per_s": 0.20,
      "max_temporal_process_stddev_m": 0.90
    },
    "stereo_identity": {
      "identity_mahalanobis_score_scale": 30.0,
      "identity_max_mahalanobis_sq": 20.0,
      "identity_swap_nll_margin": 7.0
    }
  }
})JSON");
    const auto runtime_math_loaded = bt::LoadConfig(runtime_math_path);
    BT_CHECK(runtime_math_loaded.ok());
    BT_CHECK_NEAR(runtime_math_loaded.value().tracking.solver_observation_weighting.reference_stddev_m, 0.06f, 1e-6f);
    BT_CHECK_NEAR(runtime_math_loaded.value().tracking.solver_observation_weighting.fallback_stddev_m, 1.20f, 1e-6f);
    BT_CHECK_NEAR(runtime_math_loaded.value().tracking.solver_observation_weighting.temporal_process_variance_m2_per_s, 0.20f, 1e-6f);
    BT_CHECK_NEAR(runtime_math_loaded.value().tracking.stereo_identity.identity_mahalanobis_score_scale, 30.0f, 1e-6f);
    BT_CHECK_NEAR(runtime_math_loaded.value().tracking.stereo_identity.identity_max_mahalanobis_sq, 20.0f, 1e-6f);
    BT_CHECK_NEAR(runtime_math_loaded.value().tracking.stereo_identity.identity_swap_nll_margin, 7.0f, 1e-6f);

    const auto invalid_runtime_math_path = TempConfigPath("bodytracker_invalid_runtime_math_config_test.json");
    WriteFile(invalid_runtime_math_path, R"JSON({
  "tracking": {
    "solver_observation_weighting": {
      "fallback_stddev_m": 0.0
    }
  }
})JSON");
    const auto invalid_runtime_math = bt::LoadConfig(invalid_runtime_math_path);
    BT_CHECK(!invalid_runtime_math.ok());
    BT_CHECK(invalid_runtime_math.status().message.find("$.tracking.solver_observation_weighting.fallback_stddev_m") != std::string::npos);

    const auto monocular_path = TempConfigPath("bodytracker_monocular_config_test.json");
    WriteFile(monocular_path, R"JSON({
  "tracking": {
    "mode": "monocular",
    "monocular": {
      "horizontal_fov_deg": 72.0,
      "user_height_m": 1.80,
      "camera_height_m": 1.15,
      "default_depth_m": 2.40,
      "depth_confidence_scale": 0.45,
      "min_keypoint_confidence": 0.07,
      "min_seed_count": 5,
      "floor_scale_assist_enabled": true,
      "floor_depth_line_spacing_m": 0.30,
      "floor_depth_line_spacing_px": 44.0,
      "floor_depth_reference_y_px": 560.0,
      "floor_depth_reference_m": 2.10,
      "floor_depth_confidence": 0.75
    }
  },
  "camera_a": {"device_index": 0, "width": 640, "height": 480, "fps": 30},
  "camera_b": {"device_index": -1, "width": 640, "height": 480, "fps": 30}
})JSON");
    const auto monocular_loaded = bt::LoadConfig(monocular_path);
    BT_CHECK(monocular_loaded.ok());
    BT_CHECK(monocular_loaded.value().tracking.mode == bt::TrackingMode::Monocular);
    BT_CHECK(monocular_loaded.value().camera_b.device_index == -1);
    BT_CHECK(monocular_loaded.value().tracking.monocular.image_width == 640);
    BT_CHECK(monocular_loaded.value().tracking.monocular.image_height == 480);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.horizontal_fov_deg, 72.0, 1e-6);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.user_height_m, 1.80, 1e-6);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.depth_confidence_scale, 0.45, 1e-6);
    BT_CHECK(monocular_loaded.value().tracking.monocular.min_seed_count == 5);
    BT_CHECK(monocular_loaded.value().tracking.monocular.floor_scale_assist_enabled);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.floor_depth_line_spacing_m, 0.30, 1e-6);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.floor_depth_line_spacing_px, 44.0, 1e-6);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.floor_depth_reference_y_px, 560.0, 1e-6);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.floor_depth_reference_m, 2.10, 1e-6);
    BT_CHECK_NEAR(monocular_loaded.value().tracking.monocular.floor_depth_confidence, 0.75, 1e-6);

    const auto invalid_floor_assist_path = TempConfigPath("bodytracker_invalid_monocular_floor_assist_config_test.json");
    WriteFile(invalid_floor_assist_path, R"JSON({
  "tracking": {
    "mode": "monocular",
    "monocular": {
      "floor_scale_assist_enabled": true,
      "floor_depth_line_spacing_m": 0.30,
      "floor_depth_line_spacing_px": 0.0
    }
  },
  "camera_a": {"device_index": 0, "width": 640, "height": 480, "fps": 30},
  "camera_b": {"device_index": -1, "width": 640, "height": 480, "fps": 30}
})JSON");
    const auto invalid_floor_assist = bt::LoadConfig(invalid_floor_assist_path);
    BT_CHECK(invalid_floor_assist.ok());
    BT_CHECK(invalid_floor_assist.value().tracking.monocular.floor_scale_assist_enabled);
    BT_CHECK_NEAR(invalid_floor_assist.value().tracking.monocular.floor_depth_line_spacing_m, 0.30, 1e-6);
    BT_CHECK_NEAR(invalid_floor_assist.value().tracking.monocular.floor_depth_line_spacing_px, 0.0, 1e-6);



    const auto monocular_ignores_camera_b_roi_path = TempConfigPath("bodytracker_monocular_ignores_camera_b_roi_config_test.json");
    WriteFile(monocular_ignores_camera_b_roi_path, R"JSON({
  "tracking": {"mode": "monocular"},
  "camera_a": {"device_index": 0, "width": 640, "height": 480, "fps": 30},
  "camera_b": {
    "device_index": -1,
    "width": 640,
    "height": 480,
    "fps": 30,
    "initial_roi_enabled": true,
    "initial_roi": [0.5, 0.5, -0.1, 0.2]
  }
})JSON");
    const auto monocular_ignores_camera_b_roi = bt::LoadConfig(monocular_ignores_camera_b_roi_path);
    BT_CHECK(monocular_ignores_camera_b_roi.ok());

        const auto invalid_mode_path = TempConfigPath("bodytracker_invalid_tracking_mode_config_test.json");
    WriteFile(invalid_mode_path, R"JSON({
  "tracking": {"mode": "single"}
})JSON");
    const auto invalid_mode = bt::LoadConfig(invalid_mode_path);
    BT_CHECK(!invalid_mode.ok());

    const auto schema_type_path = TempConfigPath("bodytracker_schema_type_config_test.json");
    WriteFile(schema_type_path, R"JSON({
  "tracking": {"max_frame_skew_ms": "slow"}
})JSON");
    const auto schema_type = bt::LoadConfig(schema_type_path);
    BT_CHECK(!schema_type.ok());
    BT_CHECK(schema_type.status().message.find("Config JSON schema validation failed") != std::string::npos);
    BT_CHECK(schema_type.status().message.find("$.tracking.max_frame_skew_ms") != std::string::npos);

    const auto schema_array_path = TempConfigPath("bodytracker_schema_array_config_test.json");
    WriteFile(schema_array_path, R"JSON({
  "osc": {
    "tracker_space_role_offsets": [[0.0, 0.0, 0.0]]
  }
})JSON");
    const auto schema_array = bt::LoadConfig(schema_array_path);
    BT_CHECK(!schema_array.ok());
    BT_CHECK(schema_array.status().message.find("$.osc.tracker_space_role_offsets") != std::string::npos);

    const auto alias_path = TempConfigPath("bodytracker_alias_precedence_config_test.json");
    WriteFile(alias_path, R"JSON({
  "web": {"enabled": false, "bind_address": "127.0.0.1", "port": 8081},
  "web_ui": {"enabled": true, "bind_address": "127.0.0.1", "port": 8082},
  "tracking": {
    "model_path": "models/tracking.onnx",
    "calibration_path": "calib/tracking.json"
  },
  "inference": {
    "model_path": "models/inference.onnx",
    "device": "directml"
  },
  "calibration_path": "calib/top_level.json"
})JSON");

    const auto alias_loaded = bt::LoadConfig(alias_path);
    BT_CHECK(alias_loaded.ok());
    // Legacy web/web_ui aliases must not affect runtime config now that server UI is gone.
    // tracking.* is now canonical; legacy inference.model_path/top-level calibration_path
    // only fill missing tracking fields.
    BT_CHECK(alias_loaded.value().tracking.model_path == std::filesystem::path("models/tracking.onnx"));
    BT_CHECK(alias_loaded.value().inference.model_path == std::filesystem::path("models/tracking.onnx"));
    BT_CHECK(alias_loaded.value().inference.device == "directml");
    BT_CHECK(alias_loaded.value().tracking.calibration_path == std::filesystem::path("calib/tracking.json"));
    BT_CHECK(alias_loaded.value().calibration_path == std::filesystem::path("calib/tracking.json"));

    const auto legacy_alias_path = TempConfigPath("bodytracker_legacy_alias_config_test.json");
    WriteFile(legacy_alias_path, R"JSON({
  "inference": {
    "model_path": "models/inference.onnx",
    "device": "cpu"
  },
  "calibration_path": "calib/top_level.json"
})JSON");
    const auto legacy_alias_loaded = bt::LoadConfig(legacy_alias_path);
    BT_CHECK(legacy_alias_loaded.ok());
    BT_CHECK(legacy_alias_loaded.value().tracking.model_path == std::filesystem::path("models/inference.onnx"));
    BT_CHECK(legacy_alias_loaded.value().inference.model_path == std::filesystem::path("models/inference.onnx"));
    BT_CHECK(legacy_alias_loaded.value().tracking.calibration_path == std::filesystem::path("calib/top_level.json"));
    BT_CHECK(legacy_alias_loaded.value().calibration_path == std::filesystem::path("calib/top_level.json"));

    const auto invalid_inference_path = TempConfigPath("bodytracker_invalid_inference_device_config_test.json");
    WriteFile(invalid_inference_path, R"JSON({
  "inference": {"device": "cuda"}
})JSON");
    const auto invalid_inference = bt::LoadConfig(invalid_inference_path);
    BT_CHECK(!invalid_inference.ok());

    const auto optional_knees_path = TempConfigPath("bodytracker_optional_knee_osc_config_test.json");
    WriteFile(optional_knees_path, R"JSON({
  "osc": {
    "pelvis_tracker_index": 1,
    "left_foot_tracker_index": 2,
    "right_foot_tracker_index": 3,
    "left_knee_tracker_index": 0,
    "right_knee_tracker_index": 0
  }
})JSON");
    const auto optional_knees = bt::LoadConfig(optional_knees_path);
    BT_CHECK(optional_knees.ok());
    BT_CHECK(optional_knees.value().osc.left_knee_tracker_index == 0);
    BT_CHECK(optional_knees.value().osc.right_knee_tracker_index == 0);

    const auto invalid_required_tracker_path = TempConfigPath("bodytracker_invalid_required_osc_tracker_config_test.json");
    WriteFile(invalid_required_tracker_path, R"JSON({
  "osc": {
    "pelvis_tracker_index": 0,
    "left_foot_tracker_index": 2,
    "right_foot_tracker_index": 3,
    "left_knee_tracker_index": 4,
    "right_knee_tracker_index": 5
  }
})JSON");
    const auto invalid_required_tracker = bt::LoadConfig(invalid_required_tracker_path);
    BT_CHECK(!invalid_required_tracker.ok());

    const auto duplicate_tracker_path = TempConfigPath("bodytracker_duplicate_osc_tracker_config_test.json");
    WriteFile(duplicate_tracker_path, R"JSON({
  "osc": {
    "pelvis_tracker_index": 1,
    "left_foot_tracker_index": 2,
    "right_foot_tracker_index": 3,
    "left_knee_tracker_index": 2,
    "right_knee_tracker_index": 0
  }
})JSON");
    const auto duplicate_tracker = bt::LoadConfig(duplicate_tracker_path);
    BT_CHECK(!duplicate_tracker.ok());

    const auto manual_source_path = TempConfigPath("bodytracker_unknown_manual_tracker_space_source_config_test.json");
    WriteFile(manual_source_path, R"JSON({
  "osc": {
    "manual_tracker_space_transform_valid": true,
    "manual_tracker_space_position_offset": [0.1, 0.2, 0.3],
    "manual_tracker_space_rotation": [0.0, 0.0, 0.0, 1.0],
    "manual_tracker_space_scale": 1.0,
    "manual_tracker_space_role_offsets": [
      [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0]
    ],
    "manual_tracker_space_source": "qr_manual_alignment"
  }
})JSON");
    const auto manual_source = bt::LoadConfig(manual_source_path);
    BT_CHECK(manual_source.ok());
    BT_CHECK(manual_source.value().osc.manual_tracker_space_transform_valid);
    BT_CHECK(manual_source.value().osc.manual_tracker_space_source == "qr_manual_alignment");

    const auto invalid_path = TempConfigPath("bodytracker_invalid_config_test.json");
    WriteFile(invalid_path, R"JSON({
  "tracking": {
    "tracker_ekf": {
      "min_measurement_variance_m2": 0.5,
      "max_measurement_variance_m2": 0.1
    }
  }
})JSON");

    const auto invalid = bt::LoadConfig(invalid_path);
    BT_CHECK(!invalid.ok());

    const auto saved_path = TempConfigPath("bodytracker_saved_default_config_test.json");
    const auto saved = bt::SaveDefaultConfig(saved_path);
    BT_CHECK(saved.ok());
    const auto loaded_default = bt::LoadConfig(saved_path);
    BT_CHECK(loaded_default.ok());
    BT_CHECK(loaded_default.value().tracking.tracker_ekf.enabled);
    BT_CHECK_NEAR(loaded_default.value().tracking.tracker_ekf.process_noise_mps2, 8.0, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.tracker_ekf.foot_orientation_gain, 0.35, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.temporal_update.free_gain, 0.75, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.temporal_update.supported_gain, 0.20, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.temporal_update.foot_free_gain, 0.55, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.temporal_update.foot_supported_gain, 0.12, 1e-6);
    BT_CHECK(loaded_default.value().tracking.motion_consistency.confirm_frames == 2);
    BT_CHECK_NEAR(loaded_default.value().tracking.motion_consistency.contact_root_correction_gain, 0.20, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.motion_consistency.contact_root_max_correction_m, 0.015, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.motion_consistency.contact_root_max_residual_m, 0.035, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.motion_consistency.contact_root_max_disagreement_m, 0.012, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.motion_consistency.contact_root_min_alignment, 0.75, 1e-6);
    BT_CHECK_NEAR(loaded_default.value().tracking.motion_consistency.contact_root_min_support_confidence, 0.75, 1e-6);

    std::filesystem::remove(valid_path);
    std::filesystem::remove(monocular_path);
    std::filesystem::remove(runtime_math_path);
    std::filesystem::remove(invalid_runtime_math_path);
    std::filesystem::remove(invalid_floor_assist_path);
    std::filesystem::remove(monocular_ignores_camera_b_roi_path);
    std::filesystem::remove(invalid_mode_path);
    std::filesystem::remove(schema_type_path);
    std::filesystem::remove(schema_array_path);
    std::filesystem::remove(invalid_path);
    std::filesystem::remove(invalid_inference_path);
    std::filesystem::remove(optional_knees_path);
    std::filesystem::remove(invalid_required_tracker_path);
    std::filesystem::remove(duplicate_tracker_path);
    std::filesystem::remove(manual_source_path);
    std::filesystem::remove(alias_path);
    std::filesystem::remove(saved_path);

    return 0;
}
