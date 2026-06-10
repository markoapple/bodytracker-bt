#!/usr/bin/env python3
"""Static wiring checks for config/status/UI paths.

These tests intentionally avoid CMake/WebView/OpenCV. They guard the recent runtime-control
layer so fields shown or saved by the UI match the C++ config/status paths.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]

CONTACT_ROOT_KEYS = (
    "contact_root_correction_gain",
    "contact_root_max_correction_m",
    "contact_root_max_residual_m",
    "contact_root_max_disagreement_m",
    "contact_root_min_alignment",
    "contact_root_min_support_confidence",
)


class RuntimeControlWiringTest(unittest.TestCase):

    def test_checked_in_default_config_matches_save_default_literal(self) -> None:
        default_config = json.loads((REPO_ROOT / "config" / "default.json").read_text(encoding="utf-8"))
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        save_block = config_cpp[config_cpp.index("Status SaveDefaultConfig"):]
        marker = 'R"JSON('
        start = save_block.index(marker) + len(marker)
        end = save_block.index(')JSON";', start)
        saved_default = json.loads(save_block[start:end])
        self.assertEqual(default_config, saved_default)

    def test_contact_root_defaults_reach_config_save_and_desktop_state(self) -> None:
        default_config = json.loads((REPO_ROOT / "config" / "default.json").read_text(encoding="utf-8"))
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")

        motion_defaults = default_config["tracking"]["motion_consistency"]
        for key in CONTACT_ROOT_KEYS:
            self.assertIn(key, motion_defaults)
            self.assertIn(f'"{key}"', config_cpp)
            self.assertIn(f'{{"{key}", cfg.tracking.motion_consistency.{key}}}', main_cpp)

    def test_desktop_save_config_writes_contact_root_fields_from_payload_or_current_config(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        save_config = main_cpp[main_cpp.index("nlohmann::json SaveConfigFromUi"):main_cpp.index("nlohmann::json OpenModelsFolder")]

        for key in CONTACT_ROOT_KEYS:
            self.assertIn(f'motion["{key}"] = payload.value("{key}", current_motion.{key});', save_config)

    def test_desktop_ui_controls_match_save_payload_and_config_state_paths(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")

        ui_ids = {
            "contactRootGain": "contact_root_correction_gain",
            "contactRootMaxCorrection": "contact_root_max_correction_m",
            "contactRootMaxResidual": "contact_root_max_residual_m",
            "contactRootMinSupport": "contact_root_min_support_confidence",
        }
        for element_id, payload_key in ui_ids.items():
            self.assertIn(f'id="{element_id}"', index_html)
            self.assertIn(f'"{element_id}"', app_js)
            self.assertIn(f"{payload_key}: Number(el.{element_id}?.value)", app_js)
            self.assertIn(f'motion.{payload_key}', app_js)

    def test_desktop_status_export_matches_replay_blame_fields_for_support_and_stereo(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")

        for required in (
            '{"support_constraints", SupportSolveConstraintsUiToJson(solver.final_constraints)}',
            '{"root", MotionTargetUiToJson(telemetry.targets[static_cast<std::size_t>(bt::MotionTarget::Root)])}',
            '{"left_foot", MotionTargetUiToJson(telemetry.targets[static_cast<std::size_t>(bt::MotionTarget::LeftFoot)])}',
            '{"right_foot", MotionTargetUiToJson(telemetry.targets[static_cast<std::size_t>(bt::MotionTarget::RightFoot)])}',
            '{"foot_disagreement_m", contact.disagreement_m}',
            '{"root_alignment", contact.root_alignment}',
        ):
            self.assertIn(required, main_cpp)

        for required in (
            '{"support_constraints", SupportSolveConstraintsToJson(solver.final_constraints)}',
            '{"foot_contact_confidence", joint.foot_contact_confidence}',
            '{"left_foot_contact_confidence", stereo.left_foot_contact_confidence}',
            '{"right_foot_contact_confidence", stereo.right_foot_contact_confidence}',
        ):
            self.assertIn(required, replay)

    def test_local_web_server_bloat_is_not_wired_into_the_app(self) -> None:
        cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        manifest = (REPO_ROOT / "vcpkg.json").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")

        self.assertNotIn("local_web_server.cpp", cmake)
        self.assertNotIn("httplib", cmake)
        self.assertNotIn("cpp-httplib", manifest)
        self.assertNotIn("LocalWebServer", main_cpp)
        self.assertNotIn('"webToggle"', app_js)
        self.assertNotIn("Local web UI", index_html)
        self.assertNotIn('"web_ui"', json.dumps(json.loads((REPO_ROOT / "config" / "default.json").read_text(encoding="utf-8"))))
        self.assertNotIn('EnsureObject(j, "web_ui")', main_cpp)


    def test_empty_replay_path_uses_single_runtime_fallback_name(self) -> None:
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        guide = (REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md").read_text(encoding="utf-8")
        doctor = (REPO_ROOT / "tools" / "live_preflight_doctor.py").read_text(encoding="utf-8")

        self.assertNotIn('"runtime.ndjson"', config_cpp)
        self.assertIn('"latest-runtime.ndjson"', main_cpp)
        self.assertIn("app.recording_dir/latest-runtime.ndjson", guide)
        self.assertIn("app.recording_dir/latest-runtime.ndjson", doctor)

    def test_runtime_config_changes_reset_filter_and_evidence_reaches_support(self) -> None:
        pipeline = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.cpp").read_text(encoding="utf-8")
        foot_support = (REPO_ROOT / "src" / "tracking" / "foot_support.cpp").read_text(encoding="utf-8")

        for key in CONTACT_ROOT_KEYS:
            self.assertIn(f"Differs(a.{key}, b.{key})", pipeline)

        self.assertIn("preliminary.value().telemetry.stereo.left_foot_contact_confidence", pipeline)
        self.assertIn("preliminary.value().telemetry.stereo.right_foot_contact_confidence", pipeline)
        self.assertIn("&left_foot_evidence", pipeline)
        self.assertIn("&right_foot_evidence", pipeline)
        self.assertIn("config.min_contact_evidence_confidence", foot_support)
        self.assertIn("reliable_contact_evidence", foot_support)

    def test_body_calibration_controls_reach_save_payload(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for element_id, payload_key in (
            ("bodyCalToggle", "body_calibration_enabled"),
            ("bodyCalAutoPersistToggle", "body_calibration_auto_persist"),
            ("bodyCalRequiredSeconds", "body_calibration_required_seconds"),
            ("bodyCalMinConfidence", "body_calibration_min_overall_confidence"),
            ("bodyCalMaxCv", "body_calibration_max_segment_cv"),
        ):
            self.assertIn(f'id="{element_id}"', index_html)
            self.assertIn(f'"{element_id}"', app_js)
            self.assertIn(payload_key, app_js)
            self.assertIn(payload_key, main_cpp)

    def test_start_button_saves_visible_ui_config_before_launching_runtime(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        start_block = app_js[app_js.index('el.startRuntime?.addEventListener("click"'):app_js.index('bindClick("stopRuntime"')]
        self.assertIn('saveVisibleConfig("Manual plank refresh before runtime start")', start_block)
        self.assertIn('sendCommand("startRuntime")', start_block)
        self.assertLess(start_block.index('saveVisibleConfig("Manual plank refresh before runtime start")'), start_block.index('sendCommand("startRuntime")'))

    def test_manual_plank_is_recomputed_before_save_and_runtime_start(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        self.assertIn('async function refreshManualPlankBeforeSave', app_js)
        self.assertIn('if (floorGeometryCleared || floorMarks.length < 3) return true;', app_js)
        self.assertIn('return applyManualFloorLines(sourceLabel, { keepDrawing: floorMarking && floorMarks.length < 4 });', app_js)
        self.assertIn('async function saveVisibleConfig', app_js)
        self.assertIn('return sendCommand("saveConfig", readPayload());', app_js)
        self.assertIn('el.saveConfig?.addEventListener("click", () => withButtonFeedback(el.saveConfig, "saveConfig", () => saveVisibleConfig("Manual plank refresh before save")));', app_js)
        self.assertIn('el.saveAdvanced?.addEventListener("click", () => withButtonFeedback(el.saveAdvanced, "saveConfig", () => saveVisibleConfig("Manual plank refresh before advanced save")));', app_js)

    def test_refresh_preview_button_is_wired_to_backend_camera_probe(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('id="refreshPreview"', index_html)
        self.assertIn('el.refreshPreview?.addEventListener("click", async () => {', app_js)
        self.assertIn('const cameraSlot = activeFloorCameraKey();', app_js)
        self.assertIn('sendCommand("refreshCameraPreview", { camera_a: cameraIndexForFloorSlot(cameraSlot), camera_slot: cameraSlot }, null, { silentReply: true })', app_js)
        self.assertIn('camera.preview = result.preview;', app_js)
        self.assertIn('renderFloorAssist(trackingMode());', app_js)
        self.assertIn('if (command == "refreshCameraPreview")', main_cpp)
        self.assertIn('nlohmann::json RefreshCameraPreview', main_cpp)
        self.assertIn('ProbeCameraIndex(camera_index)', main_cpp)
        self.assertIn('{"preview", probe.preview_data_url}', main_cpp)
        self.assertIn('{"width", probe.width}', main_cpp)
        self.assertIn('{"height", probe.height}', main_cpp)

    def test_save_config_validates_temp_file_before_replacing_live_config(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        save_config = main_cpp[main_cpp.index("nlohmann::json SaveConfigFromUi"):main_cpp.index("nlohmann::json OpenModelsFolder")]
        self.assertIn('temp_config_path += ".tmp"', save_config)
        self.assertIn('bt::LoadConfig(temp_config_path)', save_config)
        self.assertIn('proposed values failed validation', save_config)
        self.assertIn('replace_from_temp(temp_config_path, config_backup, "config"', save_config)
        self.assertIn('std::filesystem::rename(temp_path, state.path, ec);', save_config)
        self.assertLess(save_config.index('bt::LoadConfig(temp_config_path)'), save_config.index('replace_from_temp(temp_config_path, config_backup, "config"'))
        self.assertNotIn('std::filesystem::copy_file(temp_config_path, config_path_', save_config)

    def test_source_sanity_build_defaults_do_not_require_native_packages(self) -> None:
        cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('option(BODYTRACKER_BUILD_FULL_TESTS "Build tests that require native runtime dependency packages such as OpenCV or nlohmann_json." OFF)', cmake)

    def test_floor_geometry_state_survives_plain_save_without_new_backend_payload(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        self.assertIn('monocular_floor_geometry_calibration_enabled: floorGeometryCleared', app_js)
        self.assertIn(': (!!floorGeometryAuto || !!current.config?.tracking?.monocular?.floor_geometry_calibration_enabled)', app_js)
        self.assertIn('floor_geometry_auto: (buildFloorGeometryByCameraPayload() || {}).camera_a || null', app_js)
        self.assertIn('floor_geometry_by_camera: buildFloorGeometryByCameraPayload()', app_js)

    def test_floor_geometry_save_does_not_invent_backend_floor_plane(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        payload = app_js[app_js.index("function buildFloorGeometryPayload"):app_js.index("function modeText")]
        self.assertIn('active.backend_owned !== true', payload)
        self.assertNotIn('floor_plane ||= { valid: true', payload)

    def test_floor_geometry_calibration_save_uses_temp_file_and_validation(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        save_config = main_cpp[main_cpp.index("nlohmann::json SaveConfigFromUi"):main_cpp.index("nlohmann::json OpenModelsFolder")]
        self.assertIn('temp_calibration_path += ".tmp"', save_config)
        self.assertIn('bt::LoadCalibration(temp_calibration_path)', save_config)
        self.assertIn('replace_from_temp(temp_calibration_path, calibration_backup, "calibration"', save_config)
        self.assertNotIn('copy_file(temp_calibration_path, calibration_path', save_config)
        self.assertIn('BackupState config_backup = make_backup_state(config_path_)', save_config)
        self.assertIn('BackupState calibration_backup = make_backup_state(calibration_path)', save_config)
        self.assertIn('backup_if_present(config_backup', save_config)
        self.assertIn('backup_if_present(calibration_backup', save_config)
        self.assertIn('restore_from_backup(config_backup', save_config)
        self.assertIn('restore_from_backup(calibration_backup', save_config)
        self.assertIn('std::filesystem::rename(state.path, state.backup, ec);', save_config)
        self.assertIn('std::filesystem::rename(temp_path, state.path, ec);', save_config)
        self.assertIn('const bool stale_backup_exists = std::filesystem::exists(state.backup, ec);', save_config)
        self.assertIn('state.backup_created = true;', save_config)
        self.assertLess(
            save_config.index('backup_if_present(config_backup'),
            save_config.index('replace_from_temp(temp_config_path, config_backup, "config"'),
        )
        self.assertLess(
            save_config.index('replace_from_temp(temp_config_path, config_backup, "config"'),
            save_config.index('replace_from_temp(temp_calibration_path, calibration_backup, "calibration"'),
        )
        self.assertNotIn('refusing UI-owned floor geometry', save_config)


    def test_projective_manual_plank_save_does_not_require_scalar_spacing_gate(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        payload = app_js[app_js.index("function buildFloorGeometryPayload"):app_js.index("function modeText")]
        self.assertIn('const projectiveQuad = !!(g.homography_valid && g.two_axis_grid_valid);', payload)
        self.assertNotIn('g.homography_valid = false;', payload)
        self.assertNotIn('g.family_a?.metric_spacing_valid && g.family_b?.metric_spacing_valid && g.two_axis_grid_valid', payload)

    def test_saved_manual_plank_geometry_accepts_typed_dimensions_without_marks(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        payload = app_js[app_js.index("function buildFloorGeometryPayload"):app_js.index("function modeText")]
        render_block = app_js[app_js.index("function render(state)"):app_js.index("function handleReply")]

        self.assertIn('function savedManualPlankEditable(g)', app_js)
        self.assertIn('const editableManualOutline = floorMarks.length >= 3 || savedManualPlankEditable(g);', payload)
        self.assertIn('g.manual_plank.width_m = spacingA;', payload)
        self.assertIn('g.manual_plank.length_m = spacingB;', payload)
        self.assertIn('if (editableManualOutline && Number.isFinite(spacingA)', payload)
        self.assertIn('if (editableManualOutline && Number.isFinite(spacingB)', payload)
        self.assertIn('setValue("floorSpacingM", Number(floorGeometryAuto.family_a.spacing_m));', render_block)
        self.assertIn('setValue("floorPlankLengthM", Number(floorGeometryAuto.family_b.spacing_m));', render_block)


    def test_floor_distortion_survives_backend_geometry_as_scoped_runtime_evidence(self) -> None:
        body_solver = (REPO_ROOT / "src" / "tracking" / "body_solver.cpp").read_text(encoding="utf-8")
        start = body_solver.index("MonocularTrackingConfig ApplyFloorGeometryToMonocularConfig")
        block = body_solver[start:body_solver.index("std::array<WeightedJointSeed, kHalpe26Count>", start)]
        self.assertIn('if (geometry.distortion.valid && geometry.distortion.confidence >= 0.20f)', block)
        self.assertIn('config.floor_distortion_correction_enabled = true', block)
        self.assertNotIn('raw_floor_geometry_coordinates_active', block)
        self.assertNotIn('different coordinate space', block)

    def test_floor_geometry_is_bound_to_calibrated_image_size(self) -> None:
        types_h = (REPO_ROOT / "src" / "calibration" / "calibration_types.h").read_text(encoding="utf-8")
        calibration_io = (REPO_ROOT / "src" / "calibration" / "calibration_io.cpp").read_text(encoding="utf-8")
        floor_calibrator = (REPO_ROOT / "src" / "calibration" / "floor_calibrator.cpp").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        pipeline = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.cpp").read_text(encoding="utf-8")
        body_solver = (REPO_ROOT / "src" / "tracking" / "body_solver.cpp").read_text(encoding="utf-8")

        self.assertIn('int image_width = 0;', types_h)
        self.assertIn('int image_height = 0;', types_h)
        self.assertIn('g.image_width = j.value("image_width", 0);', calibration_io)
        self.assertIn('g.image_height = j.value("image_height", 0);', calibration_io)
        self.assertIn('{"image_width", g.image_width}', calibration_io)
        self.assertIn('{"image_height", g.image_height}', calibration_io)
        self.assertIn('out.image_width = image_width;', floor_calibrator)
        self.assertIn('out.image_height = image_height;', floor_calibrator)
        self.assertIn('g.image_width = image_w;', main_cpp)
        self.assertIn('g.image_height = image_h;', main_cpp)
        self.assertIn('SanitizeMonocularFloorGeometryForImageSpace', pipeline)
        self.assertIn('FloorGeometryUsesRawImageSpace', pipeline)
        self.assertIn('return !FloorGeometryUsesRawImageSpace(geometry);', pipeline)
        self.assertIn('floor_geometry_image_size_mismatch_saved_', pipeline)
        self.assertIn('FloorGeometryImageSpaceMatches', body_solver)
        self.assertIn('FloorGeometryUsesRawImageSpace', body_solver)
        self.assertIn('return !FloorGeometryUsesRawImageSpace(geometry);', body_solver)
        self.assertIn('config.floor_projective_homography_enabled = false', body_solver)

    def test_floor_geometry_runtime_reports_sanitized_geometry_and_actual_depth_use(self) -> None:
        pipeline = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.cpp").read_text(encoding="utf-8")
        body_solver = (REPO_ROOT / "src" / "tracking" / "body_solver.cpp").read_text(encoding="utf-8")
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")

        step_block = pipeline[pipeline.index("Result<TrackingPipelineSnapshot> TrackingPipeline::Step"):pipeline.index("Result<TrackingPipelineSnapshot> TrackingPipeline::SolveFromRecordedTrackers")]
        self.assertIn("solve_inputs.floor_geometry = has_monocular_floor_geometry_status", step_block)
        self.assertIn("calibration_.camera_a_floor_geometry.valid ? calibration_.camera_a_floor_geometry : calibration_.floor_geometry", step_block)
        self.assertIn("calibration_.camera_b_floor_geometry", step_block)
        self.assertIn("inputs.camera_a_image_width > 0 ? inputs.camera_a_image_width", step_block)
        self.assertIn("inputs.camera_b_image_width > 0 ? inputs.camera_b_image_width", step_block)
        self.assertIn("snapshot_.floor_geometry = solve_inputs.floor_geometry;", step_block)
        self.assertNotIn("snapshot_.floor_geometry = calibration_.floor_geometry;", step_block)

        self.assertIn("const bool geometry_depth_used = measurements.value().scale_source == MonocularScaleSource::FloorProjective", body_solver)
        self.assertIn("measurements.value().scale_source == MonocularScaleSource::FloorSpacing", body_solver)
        self.assertIn("telemetry->floor_geometry_used = inputs.floor_geometry.valid && geometry_depth_used;", body_solver)
        self.assertIn("telemetry->floor_geometry_confidence = telemetry->floor_geometry_used", body_solver)
        self.assertIn('{"image_width", g.image_width}', replay)
        self.assertIn('{"image_height", g.image_height}', replay)

    def test_floor_geometry_save_preserves_payload_and_runtime_sanitizes_stale_image_space(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        save_config = main_cpp[main_cpp.index("nlohmann::json SaveConfigFromUi"):main_cpp.index("nlohmann::json OpenModelsFolder")]

        self.assertIn("floor_geometry_uses_raw_image_space", save_config)
        self.assertIn("Runtime sanitizes stale raw", save_config)
        self.assertNotIn("refusing raw image-space floor geometry", save_config)
        self.assertNotIn("refusing stale floor geometry: saved image binding", save_config)
        self.assertIn('monocular["floor_projective_homography_enabled"] = payload.value("monocular_floor_projective_homography_enabled", current_monocular.floor_projective_homography_enabled);', save_config)
        self.assertIn('monocular["floor_from_image"] = Mat3ToJson(current_monocular.floor_from_image);', save_config)
        self.assertIn('monocular["floor_distortion_correction_enabled"] = payload.value("monocular_floor_distortion_correction_enabled", current_monocular.floor_distortion_correction_enabled);', save_config)
        self.assertIn('monocular["floor_camera_orientation_enabled"] = payload.value("monocular_floor_camera_orientation_enabled", current_monocular.floor_camera_orientation_enabled);', save_config)

    def test_wall_rectangles_are_persisted_and_used_as_runtime_orientation_evidence(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        calibration_types = (REPO_ROOT / "src" / "calibration" / "calibration_types.h").read_text(encoding="utf-8")
        calibration_io = (REPO_ROOT / "src" / "calibration" / "calibration_io.cpp").read_text(encoding="utf-8")
        pipeline = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.cpp").read_text(encoding="utf-8")
        body_solver = (REPO_ROOT / "src" / "tracking" / "body_solver.cpp").read_text(encoding="utf-8")

        for required in (
            "function buildWallRectanglesPayload()",
            "wall_rectangles_auto: buildWallRectanglesPayload()",
            "wall_rectangles_by_camera: buildWallRectanglesByCameraPayload()",
            "function hydrateWallRectanglesFromStatus()",
            "current.calibration?.wall_rectangles_by_camera",
        ):
            self.assertIn(required, app_js)
        self.assertIn("std::vector<WallRectangleCalibration> wall_rectangles{}", calibration_types)
        self.assertIn("std::vector<WallRectangleCalibration> camera_a_wall_rectangles{}", calibration_types)
        self.assertIn("std::vector<WallRectangleCalibration> camera_b_wall_rectangles{}", calibration_types)
        self.assertIn('{"wall_rectangles", WallRectanglesToJson(bundle.wall_rectangles)}', calibration_io)
        self.assertIn('{"wall_rectangles_by_camera"', calibration_io)
        self.assertIn('calibration_json["wall_rectangles"] = std::move(walls);', main_cpp)
        self.assertIn('calibration_json["wall_rectangles_by_camera"] = std::move(by_camera);', main_cpp)
        self.assertIn('{"wall_rectangles", WallRectanglesToJson(bundle.wall_rectangles)}', main_cpp)
        self.assertIn("solve_inputs.wall_rectangles = config_.mode == TrackingMode::Monocular", pipeline)
        self.assertIn("solve_inputs.camera_a_wall_rectangles = camera_a_wall_rectangles;", pipeline)
        self.assertIn("solve_inputs.camera_b_wall_rectangles = config_.mode == TrackingMode::Stereo", pipeline)
        self.assertIn("SanitizeWallRectanglesForImageSpace", pipeline)
        self.assertIn("ApplyWallRectanglesToMonocularConfig(", body_solver)
        self.assertIn("WallRectangleImageSpaceMatches(wall, config)", body_solver)
        self.assertIn("geometry_stereo_status", body_solver)
        self.assertIn("partial_not_used", body_solver)
        self.assertIn("config.floor_camera_pitch_rad = wall_pitch;", body_solver)
        self.assertIn("config.floor_camera_roll_rad = wall_roll;", body_solver)

    def test_backend_geometry_supersedes_and_clears_stale_monocular_fields(self) -> None:
        body_solver = (REPO_ROOT / "src" / "tracking" / "body_solver.cpp").read_text(encoding="utf-8")
        helper = body_solver[body_solver.index("void ClearBackendFloorGeometryRuntimeFields"):body_solver.index("bool FloorGeometryUsesRawImageSpace")]
        apply = body_solver[body_solver.index("MonocularTrackingConfig ApplyFloorGeometryToMonocularConfig"):body_solver.index("std::array<WeightedJointSeed, kHalpe26Count> BuildMonocularSeeds")]

        for required in (
            "config.floor_projective_homography_enabled = false;",
            "config.floor_distortion_correction_enabled = false;",
            "config.floor_camera_orientation_enabled = false;",
            "config.floor_projective_confidence = 0.0f;",
        ):
            self.assertIn(required, helper)
        self.assertIn("ClearBackendFloorGeometryRuntimeFields(config);", apply)
        self.assertIn("if (!geometry.valid)", apply)
        self.assertIn("FloorGeometryWasRejectedForRuntimeImageSpace(geometry)", apply)
        self.assertIn("const bool backend_metric_replacement", apply)
        self.assertIn("if (backend_metric_replacement)", apply)
        self.assertIn("ClearLegacyScalarFloorAssist(config);", apply)
        self.assertIn("if (!FloorGeometryImageSpaceMatches(geometry, config))", apply)

    def test_ui_hydrates_projective_status_from_saved_calibration_not_config_flag(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        render_block = app_js[app_js.index("function render(state)"):app_js.index("function handleReply")]
        status_block = app_js[app_js.index("function currentFloorAssistStatus"):app_js.index("function renderFloorAssist")]

        self.assertIn('if (current.calibration?.floor_geometry)', render_block)
        self.assertIn("acceptedProjectiveFloorGeometry", status_block)
        self.assertIn("acceptedScalarFloorGeometry", status_block)
        self.assertIn("geometryAccepted", status_block)
        self.assertIn('const acceptedSource = projectiveAccepted ? "floor_projective" : (scalarAccepted ? "floor_spacing" : "--");', app_js)

    def test_saved_projective_geometry_is_accepted_runtime_geometry_not_standby_until_use(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        status_block = app_js[app_js.index("function currentFloorAssistStatus"):app_js.index("function renderFloorAssist")]
        render_block = app_js[app_js.index("function renderFloorAssist"):app_js.index("function cameraPointFromPreviewEvent")]

        self.assertIn("acceptedProjectiveFloorGeometry", status_block)
        self.assertIn("geometryAccepted", status_block)
        self.assertIn('return mode === "stereo" && !stereoFallbackActive() ? "standby" : "active";', status_block)
        self.assertNotIn('runtime has not consumed it yet', render_block)
        self.assertIn('accepted runtime geometry', render_block)
        self.assertIn('used this frame', render_block)
        self.assertIn('!checked("floorAssistToggle") && !projectiveAccepted && !scalarAccepted', render_block)

    def test_runtime_rejected_geometry_is_not_reported_as_valid_snapshot_geometry(self) -> None:
        pipeline = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.cpp").read_text(encoding="utf-8")
        sanitize = pipeline[pipeline.index("FloorGeometryCalibration SanitizeMonocularFloorGeometryForImageSpace"):pipeline.index("LowerBodyModel ApplyMonocularUserScale")]
        recorded = pipeline[pipeline.index("Result<TrackingPipelineSnapshot> TrackingPipeline::SolveFromRecordedTrackers"):pipeline.index("BodyCalibrationTelemetry TrackingPipeline::PersistBodyCalibrationOnShutdown")]
        shutdown = pipeline[pipeline.index("BodyCalibrationTelemetry TrackingPipeline::PersistBodyCalibrationOnShutdown"):pipeline.index("TrackingPipelineSnapshot TrackingPipeline::Snapshot")]

        self.assertIn("geometry.valid = false;", sanitize)
        self.assertIn("geometry.family_a.valid = false;", sanitize)
        self.assertIn("geometry.family_b.valid = false;", sanitize)
        self.assertIn("geometry.homography_valid = false;", sanitize)
        self.assertIn("geometry.floor_from_image = identity;", sanitize)
        self.assertIn("geometry.image_from_floor = identity;", sanitize)
        self.assertIn("geometry.camera_orientation_valid = false;", sanitize)
        self.assertIn("geometry.distortion.applied_to_runtime = false;", sanitize)
        self.assertIn('snapshot_.floor_geometry = FloorGeometryCalibration{};', recorded)
        self.assertIn('not_used_replay_tracker_input', recorded)
        self.assertNotIn('snapshot_.floor_geometry = calibration_.floor_geometry;', shutdown)

    def test_yaw_only_geometry_is_not_camera_orientation_runtime_input(self) -> None:
        floor_calibrator = (REPO_ROOT / "src" / "calibration" / "floor_calibrator.cpp").read_text(encoding="utf-8")
        yaw_only = floor_calibrator[floor_calibrator.index("No real vanishing point"):floor_calibrator.index("EstimateFloorGeometryCalibration", floor_calibrator.index("No real vanishing point"))]
        self.assertIn("yaw is line-family evidence", yaw_only)
        self.assertIn("out.camera_orientation_valid = false;", yaw_only)
        self.assertIn("out.camera_orientation_confidence = 0.0f;", yaw_only)


    def test_accepted_runtime_geometry_is_only_consumable_depth_geometry(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")
        for source in (main_cpp, replay):
            scalar = source[source.index("bool AcceptedRuntimeScalarFloorGeometry"):source.index("bool AcceptedRuntimeFloorGeometry")]
            accepted = source[source.index("bool AcceptedRuntimeFloorGeometry"):source.index("std::string AcceptedFloorGeometrySource")]
            self.assertIn("family_a.valid", scalar)
            self.assertIn("family_a.metric_spacing_valid", scalar)
            self.assertNotIn("family_b.metric_spacing_valid", accepted)
            self.assertNotIn("floor_plane.valid", accepted)
            self.assertIn("AcceptedRuntimeProjectiveFloorGeometry", accepted)
            self.assertIn("AcceptedRuntimeScalarFloorGeometry", accepted)

    def test_backend_solved_geometry_is_not_status_downgraded_by_confidence(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")
        runtime_status = main_cpp[main_cpp.index("std::string FloorAssistRuntimeStatus"):main_cpp.index("const bt::TrackingSolverTelemetry& EffectiveSolverTelemetry")]
        replay_status = replay[replay.index("const char* MonocularFloorAssistStatus"):replay.index("TrackingSolverTelemetry EffectiveSolverTelemetry")]
        ui_status = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        ui_status = ui_status[ui_status.index("function currentFloorAssistStatus"):ui_status.index("function renderFloorAssist")]
        self.assertIn('return "active";', runtime_status)
        self.assertIn('return accepted_runtime_geometry ? "standby" : config_status;', runtime_status)
        self.assertIn('return "active";', replay_status)
        self.assertIn('return "standby";', replay_status)
        self.assertNotIn('? "active" : "weak"', runtime_status)
        self.assertNotIn('? "active" : "weak"', replay_status)
        self.assertNotIn('confidence < 0.20', ui_status)

    def test_replay_floor_assist_status_includes_projective_runtime_use(self) -> None:
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")
        status = replay[replay.index("const char* MonocularFloorAssistStatus"):replay.index("TrackingSolverTelemetry EffectiveSolverTelemetry")]
        self.assertIn("MonocularScaleSource::FloorSpacing", status)
        self.assertIn("MonocularScaleSource::FloorProjective", status)
        self.assertIn("stereo.floor_geometry_used", status)
        self.assertIn("AcceptedRuntimeFloorGeometry", status)
        self.assertIn("floor_geometry_accepted", replay)

    def test_single_manual_end_cap_is_not_promoted_to_a_second_line_family(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('g.family_count = has_cap_b ? 2 : 1;', main_cpp)
        self.assertIn('g.family_b.valid = full_quad;', main_cpp)
        self.assertIn('g.family_b.confidence = full_quad ? confidence : 0.0f;', main_cpp)
        self.assertIn('g.family_b.accepted_line_count = full_quad ? 2 : 0;', main_cpp)
        self.assertIn('single_end_cap_geometry_recorded_in_manual_plank_not_a_line_family', main_cpp)


    def test_manual_plank_width_is_degraded_scalar_metric_evidence(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        manual = main_cpp[main_cpp.index("Manual plank width is metric evidence"):main_cpp.index("nlohmann::json plank = {")]
        scalar = app_js[app_js.index("function scalarFloorSpacingAllowed"):app_js.index("function buildFloorGeometryPayload")]

        self.assertIn("const bool scalar_spacing_usable = width_metric_valid;", manual)
        self.assertIn("g.family_a.metric_spacing_valid = width_metric_valid;", manual)
        self.assertIn("g.floor_plane.valid = metric_scale_usable;", manual)
        self.assertIn("manual_plank_width_metric_evidence", manual)
        self.assertIn("manual_plank_width_metric_assist", manual)
        self.assertNotIn('source.includes("manual_plank")', scalar)

    def test_inference_device_allows_cpu_and_directml_and_reaches_runtime_status(self) -> None:
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        session_h = (REPO_ROOT / "src" / "inference" / "rtmpose_session.h").read_text(encoding="utf-8")
        session_cpp = (REPO_ROOT / "src" / "inference" / "rtmpose_session.cpp").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        default_config = json.loads((REPO_ROOT / "config" / "default.json").read_text(encoding="utf-8"))
        self.assertEqual(default_config["inference"]["device"], "directml")
        self.assertIn('cfg.inference.device != "cpu" && cfg.inference.device != "directml" && cfg.inference.device != "directml_strict"', config_cpp)
        self.assertIn('inference.device must be cpu, directml, or directml_strict', config_cpp)
        self.assertIn("std::string active_device", session_h)
        self.assertIn("bool ep_fallback", session_h)
        self.assertIn("OrtSessionOptionsAppendExecutionProvider_DML", session_cpp)
        self.assertIn("DirectML session creation failed; falling back to CPU", session_cpp)
        self.assertIn("model.Load(cfg.tracking.model_path, cfg.inference.device)", main_cpp)
        self.assertIn('{"model_active_device", debug.model_active_device}', main_cpp)
        self.assertIn('{"model_ep_fallback", debug.model_ep_fallback}', main_cpp)
        self.assertIn("current.model?.active_device", app_js)
        self.assertIn("current.model?.ep_fallback", app_js)

    def test_vrchat_tracker_roles_are_eight_wide_through_osc_ui_debug_and_replay(self) -> None:
        tracker_h = (REPO_ROOT / "src" / "tracking" / "tracker_synthesis.h").read_text(encoding="utf-8")
        osc_h = (REPO_ROOT / "src" / "io" / "osc_sender.h").read_text(encoding="utf-8")
        osc_cpp = (REPO_ROOT / "src" / "io" / "osc_sender.cpp").read_text(encoding="utf-8")
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")
        replay_player = (REPO_ROOT / "src" / "debug" / "replay_player.cpp").read_text(encoding="utf-8")
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")

        self.assertIn("inline constexpr std::size_t kTrackerPoseCount = 8;", tracker_h)
        self.assertIn("inline constexpr std::array<TrackerRole, kTrackerPoseCount> kTrackerRoles", tracker_h)
        for role in ("Pelvis", "LeftFoot", "RightFoot", "Chest", "LeftElbow", "RightElbow", "LeftKnee", "RightKnee"):
            self.assertIn(f"TrackerRole::{role}", tracker_h)

        for field in (
            "pelvis_tracker_index",
            "left_foot_tracker_index",
            "right_foot_tracker_index",
            "chest_tracker_index",
            "left_elbow_tracker_index",
            "right_elbow_tracker_index",
            "left_knee_tracker_index",
            "right_knee_tracker_index",
        ):
            self.assertIn(field, config_cpp)
            self.assertIn(field, main_cpp)

        self.assertIn("upper-body and knee tracker indices may be 0..8", config_cpp)
        self.assertIn("std::array<std::string, kTrackerPoseCount> position_addresses_", osc_h)
        self.assertIn("std::array<OscRoleSendState, kTrackerPoseCount> roles", osc_h)
        self.assertIn("for (const auto role : kTrackerRoles)", osc_cpp)
        self.assertIn('last_report_.status = "disabled"', osc_cpp)
        self.assertIn('role_report.reason = "unmapped"', osc_cpp)
        self.assertIn('role_report.reason = !IsFinite(vr_pose.position) ? "nonfinite_tracker_space_position" : "invalid_tracker_space_orientation"', osc_cpp)

        for element_id in ("pelvisTracker", "leftFootTracker", "rightFootTracker", "chestTracker", "leftElbowTracker", "rightElbowTracker", "leftKneeTracker", "rightKneeTracker"):
            self.assertIn(f'id="{element_id}"', index_html)
            self.assertIn(f'"{element_id}"', app_js)
        self.assertIn('id="chestTracker" type="number" min="0" max="8"', index_html)
        self.assertIn('id="leftElbowTracker" type="number" min="0" max="8"', index_html)
        self.assertIn('id="rightElbowTracker" type="number" min="0" max="8"', index_html)
        self.assertIn('id="leftKneeTracker" type="number" min="0" max="8"', index_html)
        self.assertIn('id="rightKneeTracker" type="number" min="0" max="8"', index_html)

        for required in (
            "CopyOscReportToDebug",
            "SendTrackersAndRecordOsc",
            '{"osc", OscDebugToJson(debug)}',
            "osc_sent_tracker_count",
            "osc_role_sent",
            "osc_role_reasons",
        ):
            self.assertIn(required, main_cpp)

        for required in (
            "OscToJson(snapshot)",
            '"sent_tracker_count"',
            '"active_roles"',
            '"roles"',
        ):
            self.assertIn(required, replay)
        for required in (
            'osc.value("sent_tracker_count", 0)',
            'osc.value("roles", nlohmann::json::array())',
            'ToString(kTrackerRoles[i])',
        ):
            self.assertIn(required, replay_player)

        self.assertIn("oscDebug.active_roles", app_js)
        self.assertIn("oscDebug.sent_tracker_count", app_js)
        self.assertIn("osc status", app_js)

    def test_body_calibration_controls_are_rendered_from_config_before_save(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        render_block = app_js[app_js.index("function render(state)"):app_js.index("function handleReply")]
        for required in (
            'const bodyCal = current.config?.tracking?.body_calibration || {};',
            'setPressed("bodyCalToggle", bodyCal.enabled === true);',
            'setPressed("bodyCalAutoPersistToggle", bodyCal.auto_persist !== false);',
            'setValue("bodyCalRequiredSeconds", bodyCal.required_seconds ?? 2.5);',
            'setValue("bodyCalMinConfidence", bodyCal.min_overall_confidence ?? 0.55);',
            'setValue("bodyCalMaxCv", bodyCal.max_segment_cv ?? 0.12);',
        ):
            self.assertIn(required, render_block)


    def test_ui_render_fallbacks_match_backend_false_defaults(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        render_block = app_js[app_js.index("function render(state)"):app_js.index("function handleReply")]
        self.assertIn('setPressed("stereoFallbackToggle", current.config?.tracking?.stereo_monocular_fallback_enabled === true);', render_block)
        self.assertIn('setPressed("bodyCalToggle", bodyCal.enabled === true);', render_block)
        self.assertIn('if (current.calibration?.floor_geometry)', render_block)
        self.assertIn('setValue("oscMinConfidence", current.config?.osc?.min_confidence ?? 0.20);', render_block)


    def test_persistent_json_saves_use_temp_backup_helper(self) -> None:
        calibration_io = (REPO_ROOT / "src" / "calibration" / "calibration_io.cpp").read_text(encoding="utf-8")
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        for text in (calibration_io, config_cpp):
            helper_block = text[text.index("Status WriteTextFileAtomically"):text.index("template <", text.index("Status WriteTextFileAtomically"))]
            self.assertIn('temp_path += ".tmp"', helper_block)
            self.assertIn('backup_path += ".bak"', helper_block)
            self.assertIn('std::filesystem::rename(path, backup_path, ec);', helper_block)
            self.assertIn('std::filesystem::rename(temp_path, path, ec);', helper_block)
            self.assertNotIn('std::filesystem::copy_file(path, backup_path', helper_block)
            self.assertNotIn('std::filesystem::copy_file(temp_path, path', helper_block)
            self.assertIn('const bool stale_backup_exists = std::filesystem::exists(backup_path, ec);', helper_block)
            self.assertIn('bool backup_created = false;', helper_block)
            self.assertIn('backup_created = true;', helper_block)
            self.assertIn('if (backup_created) {', helper_block)
            self.assertNotIn('const bool backup_exists = std::filesystem::exists(backup', helper_block)
        self.assertIn('return WriteTextFileAtomically(path, j.dump(2)', calibration_io)
        self.assertIn('return WriteTextFileAtomically(path, std::string(kTemplate)', calibration_io)
        self.assertIn('return WriteTextFileAtomically(path, std::string(kDefaultConfig)', config_cpp)
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn("bt::Status WriteJsonFileAtomically", main_cpp)
        self.assertIn('return WriteJsonFileAtomically(config_path, j, "camera selection config");', main_cpp)
        self.assertIn('const auto saved_config = WriteJsonFileAtomically(config_path_, j, "OSC config");', main_cpp)
        self.assertIn('const bool target_exists = std::filesystem::exists(path, ec);', main_cpp)
        self.assertIn('const bool stale_backup_exists = std::filesystem::exists(backup_path, ec);', main_cpp)
        self.assertIn('bool backup_created = false;', main_cpp)
        self.assertIn('if (ec) {', main_cpp)
        self.assertIn('if (error) {', main_cpp)
        self.assertIn('std::filesystem::rename(path, backup_path, ec);', main_cpp)
        self.assertIn('std::filesystem::rename(temp_path, path, ec);', main_cpp)
        self.assertNotIn('std::filesystem::copy_file(path, backup_path', main_cpp)
        self.assertNotIn('std::filesystem::copy_file(temp_path, path', main_cpp)
        self.assertNotIn('std::filesystem::copy_file(temp_config_path, config_path_', main_cpp)
        self.assertNotIn('std::filesystem::copy_file(temp_calibration_path, calibration_path', main_cpp)
        self.assertNotIn("std::ofstream out(config_path);", main_cpp)

    def test_ui_filesystem_status_paths_use_non_throwing_error_code_checks(self) -> None:
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('std::filesystem::exists(config_.tracking.model_path, model_exists_ec)', main_cpp)
        self.assertIn('"inspect_error", model_exists_ec ? model_exists_ec.message() : std::string()', main_cpp)
        self.assertIn('std::filesystem::exists(config_.tracking.calibration_path, calibration_exists_ec)', main_cpp)
        self.assertIn('failed to inspect calibration path', main_cpp)
        self.assertIn('std::filesystem::exists(config_path_, ec)', main_cpp)
        self.assertIn('std::filesystem::create_directories(dir, ec)', main_cpp)
        self.assertNotIn('std::filesystem::create_directories(dir);', main_cpp)

    def test_body_calibration_persistence_reaches_shutdown_snapshot_and_ui(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        pipeline_h = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.h").read_text(encoding="utf-8")
        pipeline_cpp = (REPO_ROOT / "src" / "tracking" / "tracking_pipeline.cpp").read_text(encoding="utf-8")
        replay = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")

        self.assertIn("PersistBodyCalibrationOnShutdown", pipeline_h)
        self.assertIn("SaveCalibrationBundle(calibration_, config_.calibration_path)", pipeline_cpp)
        self.assertIn("pipeline.PersistBodyCalibrationOnShutdown()", main_cpp)
        self.assertIn('{"persist_status", telemetry.persist_status}', main_cpp)
        self.assertIn('{"persist_status", telemetry.persist_status}', replay)
        self.assertIn('id="bodyCalReadout"', index_html)
        self.assertIn("bodyCalibrationStatus.persist_status", app_js)
        self.assertIn("bodyPersistStatus", app_js)

    def test_desktop_local_draft_blocks_state_refresh_until_successful_save(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        self.assertIn("let localDraftDirty = false;", app_js)
        self.assertIn("function markDraftDirty()", app_js)
        self.assertIn("localDraftDirty = true;", app_js)
        self.assertIn("syncInputs = false;", app_js)
        self.assertIn("function clearDraftDirty()", app_js)
        self.assertIn('if (ok && entry.command === "saveConfig")', app_js)
        toggle_block = app_js[app_js.index("function bindToggle"):app_js.index("function bindEvents")]
        self.assertIn("markDraftDirty();", toggle_block)
        focus_block = app_js[app_js.index('document.addEventListener("input"'):app_js.index('bindClick("scanCams"')]
        self.assertIn("markDraftDirty();", focus_block)
        self.assertIn("syncInputs = !localDraftDirty && !editingFormControl();", focus_block)

    def test_desktop_ui_actions_have_immediate_tactile_feedback(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        styles = (REPO_ROOT / "src" / "ui" / "app" / "styles.css").read_text(encoding="utf-8")

        for required in (
            "function setButtonBusy",
            "function setButtonResult",
            "function withButtonFeedback",
            "function markControlTouched",
            "function buttonIsBusy",
            'button.dataset.feedback = "busy"',
            'button.dataset.feedback = ok ? "ok" : "fail"',
            'setCommandStatus(`${label}: ${checked(id) ? "ON" : "OFF"} · save config to apply`);',
            'el.floorAssistPreview?.classList.add("is-marking")',
            'el.floorAssistPreview?.classList.add("line-started")',
            'el.floorAssistPreview?.addEventListener("pointermove", handleFloorPreviewPointerMove)',
            'floorGeometryAcceptedForRuntime(floorGeometryAuto)',
            'captured degraded metric geometry',
        ):
            self.assertIn(required, app_js)

        for required in (
            'button[data-feedback="busy"]',
            'button[data-feedback="ok"]',
            'button[data-feedback="fail"]',
            '.floor-preview-img.is-marking',
            '.floor-preview-img.line-started',
            'body.draft-dirty .command-status',
            'border-radius: 0;',
        ):
            self.assertIn(required, styles)

    def test_manual_plank_preview_mapping_is_not_cropped_or_invisible(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        styles = (REPO_ROOT / "src" / "ui" / "app" / "styles.css").read_text(encoding="utf-8")

        for required in (
            'id="floorMarkOverlay"',
            'function previewContentRect()',
            'const scale = Math.min(rect.width / imageWidth, rect.height / imageHeight);',
            'if (x < 0 || y < 0 || x > content.width || y > content.height) return null;',
            'function updateFloorMarkOverlay()',
            'floorMarkImageWidth',
            'floorMarkImageHeight',
            'Manual plank image size changed',
            'id="wallRectSlot"',
            'id="wallSlotButtons"',
            'id="wallApplySelected"',
            'const wallRectangleSlotCount = 3;',
            'function setActiveWallSlot(index)',
            'button[data-wall-slot]',
            'wall_aspect_ratio',
            'applyManualWallRectangle(wallSlotLabel())',
            'cropDrawing = false;',
        ):
            self.assertIn(required, index_html + app_js)

        self.assertIn('object-fit: contain;', styles)
        self.assertIn('.floor-mark-overlay', styles)
        self.assertIn('.floor-mark-overlay line.preview-line', styles)
        self.assertIn('.floor-mark-overlay .wall-rect.active', styles)
        self.assertIn('.floor-mark-overlay .wall-rect.solved', styles)
        self.assertIn('.wall-slot.active', styles)
        self.assertIn('vector-effect: non-scaling-stroke;', styles)
        self.assertNotIn('object-fit: cover;', styles[styles.index('.floor-preview-img'):styles.index('.floor-preview-img.is-marking')])


    def test_wall_sample_ui_behaves_like_candidate_bank_not_three_required_walls(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        styles = (REPO_ROOT / "src" / "ui" / "app" / "styles.css").read_text(encoding="utf-8")

        for required in (
            'Monocular geometry assists',
            'Sample 1',
            'start here',
            'Sample 2',
            'optional',
            'Draw wall sample',
            'Solve sample',
            'Clear sample',
        ):
            self.assertIn(required, index_html)

        for required in (
            'let activeWallCameraKey = activeFloorCameraKey();',
            'const key = activeWallCameraKey || activeFloorCameraKey();',
            'wallSlotIndexFromGeometry(geometry)',
            'return `Sample ${Number(index) + 1}`;',
            'first trace the measured width edge',
            'wallSampleCapabilityLabel(slot.geometry)',
            'wallCandidateSummary("camera_a")',
            'wallCandidateSummary("camera_b")',
            'cameraConfigForFloorSlot();',
            'cropEditPreviewActive() ? selectedCameraA() : cameraConfigForFloorSlot()',
            'usedSource === "wall_depth"',
            'active · wall sample depth',
        ):
            self.assertIn(required, app_js)

        reset_block = app_js[app_js.index("function resetFloorMarks"):app_js.index("function editingFormControl")]
        self.assertNotIn("clearAllWallRectangles();", reset_block)
        self.assertIn('.wall-slot.reserve', styles)
        self.assertIn('.wall-slot.has-depth', styles)
        self.assertIn('grid-template-columns: 1.35fr repeat(2, minmax(0, .85fr));', styles)
        self.assertNotIn('Draw wall rectangle failed', app_js)
        self.assertNotIn('Wall rectangle image size changed', app_js)

    def test_tactile_feedback_uses_clean_labels_and_cancellable_draw_state(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")

        for required in (
            'function controlLabel(node)',
            'node.querySelector?.(":scope > span")?.textContent?.trim()',
            'const label = controlLabel(node);',
            'function resetFloorMarks(clearGeometry = false)',
            'if (floorGeometryCleared) return false;',
            'setPressed("floorAssistToggle", false);',
            'setButtonResult(button, false, "draw at least 3 lines first")',
            'Draw one plank failed: refresh ${activeFloorCameraLabel()} preview first',
            'setPointerCapture',
            'releasePointerCapture',
            'function cancelFloorPreviewLine()',
            'floorLineEnd = null;',
            'pointercancel',
        ):
            self.assertIn(required, app_js)

        self.assertNotIn('const label = baseButtonLabel(node).replace', app_js)

    def test_saved_manual_geometry_status_checks_image_binding_before_claiming_active(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        status_block = app_js[app_js.index("function floorGeometryMatchesCurrentImageSize"):app_js.index("function renderFloorAssist")]

        for required in (
            'function floorGeometryMatchesCurrentImageSize(g)',
            'const geometryWidth = Number(g.image_width || 0);',
            'Math.round(geometryWidth) === Math.round(imageWidth)',
            'floorGeometryMatchesCurrentImageSize(g)',
            'const geometryAvailableButWrongImage',
            'if (geometryAvailableButWrongImage) return "invalid";',
        ):
            self.assertIn(required, status_block)

        render_block = app_js[app_js.index("function renderFloorAssist"):app_js.index("function previewImagePixelSize")]
        self.assertIn('saved floor geometry image size', render_block)
        self.assertIn('does not match current', render_block)

    def test_overview_floor_status_uses_current_ui_floor_contract(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        render_block = app_js[app_js.index("function render(state)"):app_js.index("const bodyCalibrationStatus")]
        self.assertIn('renderFloorAssist(mode);', render_block)
        self.assertIn('const floorOverviewStatus = currentFloorAssistStatus(mode);', render_block)
        self.assertLess(render_block.index('renderFloorAssist(mode);'), render_block.index('const floorOverviewStatus = currentFloorAssistStatus(mode);'))


    def test_recent_tracking_feature_smoke_wiring_is_minimal(self) -> None:
        """Recent phased features are guarded here only as smoke wiring.

        Numerical behavior belongs in the C++ component tests. This test should
        not assert comments, exact implementation phrases, or local variable
        names; those were useful during patch bring-up but are too brittle for
        long-term validation.
        """
        cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("src/tracking/epipolar_geometry.cpp", cmake)
        self.assertIn("tests/epipolar_geometry_test.cpp", cmake)
        self.assertIn("tests/triangulation_confidence_test.cpp", cmake)
        self.assertIn("tests/identity_assignment_test.cpp", cmake)
        self.assertFalse((REPO_ROOT / "src" / "tracking" / "epipolar.cpp").exists())
        self.assertFalse((REPO_ROOT / "src" / "tracking" / "epipolar.h").exists())

    def test_runtime_debug_surfaces_epipolar_decisions_without_claiming_semantics_here(self) -> None:
        """Keep one surface smoke check; branch truth is covered by component tests."""
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        replay_log = (REPO_ROOT / "src" / "debug" / "replay_log.cpp").read_text(encoding="utf-8")

        for key in (
            "epipolar_checked_count",
            "epipolar_hard_mismatch_count",
            "epipolar_pair_rejected_count",
            "epipolar_degraded_pair_softened_count",
            "mean_epipolar_error_px",
            "mean_epipolar_confidence",
        ):
            self.assertIn(key, main_cpp)
            self.assertIn(key, app_js)
            self.assertIn(key, replay_log)



    def test_alignment_authority_when_runtime_running_comes_from_debug_snapshot(self) -> None:
        """Runtime debug is the source of truth for an already-running runtime
        alignment, but an active desktop calibration session owns the wizard
        state and must not be overwritten by an unrelated runtime snapshot.
        """
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        runtime_state_h = (REPO_ROOT / "src" / "web" / "runtime_state.h").read_text(encoding="utf-8")

        # The controller must have the runtime_running_ guard before using
        # the debug snapshot's alignment.
        self.assertIn("!steamvr_session_active && runtime_running_ && debug.steamvr_alignment_recorded", main_cpp)
        self.assertIn("steamvr_status = debug.steamvr_alignment;", main_cpp)
        # The debug snapshot alignment must be used for the tracker_space
        # state computation, not the controller's stale manager.
        self.assertIn("TrackerSpaceStateToJson(config_, &steamvr_status)", main_cpp)

        # WebRuntimeState must not have a public config field that bypasses
        # the Snapshot() mechanism. The WebRuntimeSnapshot struct correctly
        # holds config{} as snapshot data; only the WebRuntimeState class
        # must not expose it directly.
        class_body = runtime_state_h[runtime_state_h.index("class WebRuntimeState"):runtime_state_h.index("using RuntimeState")]
        self.assertNotIn("AppConfig config", class_body)
        # Snapshot() must be the only way to read config state.
        self.assertIn("WebRuntimeSnapshot Snapshot() const", runtime_state_h)

    def test_web_runtime_state_config_is_private(self) -> None:
        """The public config field on WebRuntimeState was removed. All reads
        must go through Snapshot() which is mutex-protected, preventing the
        desktop controller from writing a stale config that the runtime reads
        without synchronization.
        """
        runtime_state_h = (REPO_ROOT / "src" / "web" / "runtime_state.h").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")

        # No direct access to a public config field in main.cpp
        self.assertNotIn("web_state.config", main_cpp)
        self.assertNotIn("runtime_state_.config", main_cpp)
        # SetConfig must still exist
        self.assertIn("void SetConfig(const AppConfig& next_config)", runtime_state_h)
        # Publish must not push a stale public config field into the snapshot.
        # Check only the WebRuntimeState class body (not the snapshot struct).
        class_body = runtime_state_h[runtime_state_h.index("class WebRuntimeState"):runtime_state_h.index("using RuntimeState")]
        self.assertNotIn("snapshot_.config = config;", class_body)

    def test_desktop_controller_alignment_status_reflects_runtime_authority(self) -> None:
        """When the runtime is running and the menu is idle, GetStateJson uses
        runtime debug alignment. During an active menu calibration session, the
        menu alignment manager remains the wizard-state authority.
        """
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        get_state = main_cpp[main_cpp.index("nlohmann::json GetStateJson()"):main_cpp.index("nlohmann::json HandleCommand")]
        # The alignment_manager_.Poll() is retained for provider freshness
        # when the runtime is NOT running, but the runtime_running_ guard
        # must override the status.
        self.assertIn("alignment_manager_.Poll()", get_state)
        self.assertIn("!steamvr_session_active && runtime_running_ && debug.steamvr_alignment_recorded", get_state)
        self.assertIn("steamvr_status = debug.steamvr_alignment;", get_state)


    def test_inference_device_coercion_passes_directml_through(self) -> None:
        """F1: 'directml' must not be coerced to 'directml_strict' or silently
        replaced. The readPayload includes guard and the render syncInputs
        guard must both accept 'directml' as a valid value.
        """
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")

        # readPayload must accept all three valid devices
        self.assertIn('"cpu", "directml", "directml_strict"].includes(el.inferenceDevice?.value)', app_js)
        # render syncInputs must accept all three valid devices
        self.assertIn('"cpu", "directml", "directml_strict"].includes(current.config?.inference?.device)', app_js)
        # C++ backend config validation must accept directml
        config_cpp = (REPO_ROOT / "src" / "core" / "config.cpp").read_text(encoding="utf-8")
        self.assertIn('cfg.inference.device != "cpu" && cfg.inference.device != "directml" && cfg.inference.device != "directml_strict"', config_cpp)

    def test_one_euro_fields_pass_through_save_payload(self) -> None:
        """F8: One-Euro fields are sealed internal knobs with no UI controls.
        readPayload must pass them through from config so saves don't silently
        zero them, and SaveConfigFromUi must write them to the motion section.
        """
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        main_cpp = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")

        # readPayload must pass through One-Euro fields
        self.assertIn("motion_one_euro_enabled", app_js)
        self.assertIn("motion_one_euro_min_cutoff_hz", app_js)
        self.assertIn("motion_one_euro_beta", app_js)
        self.assertIn("motion_one_euro_d_cutoff_hz", app_js)

        # SaveConfigFromUi must write them to the motion JSON
        self.assertIn('motion["one_euro_enabled"] = payload.value("motion_one_euro_enabled"', main_cpp)
        self.assertIn('motion["one_euro_min_cutoff_hz"] = payload.value("motion_one_euro_min_cutoff_hz"', main_cpp)
        self.assertIn('motion["one_euro_beta"] = payload.value("motion_one_euro_beta"', main_cpp)
        self.assertIn('motion["one_euro_d_cutoff_hz"] = payload.value("motion_one_euro_d_cutoff_hz"', main_cpp)

    def test_steamvr_finish_button_requires_canonical_required_samples(self) -> None:
        """F5: The finish button must derive its state from backend-owned
        sample keys and pending command/session state, not a local click count.
        """
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        header = (REPO_ROOT / "src" / "tracking" / "steamvr_alignment.h").read_text(encoding="utf-8")
        json_header = (REPO_ROOT / "src" / "tracking" / "steamvr_alignment_json.h").read_text(encoding="utf-8")

        self.assertIn("inline const char* LandmarkKey", header)
        self.assertIn('{"landmark_key", LandmarkKey(sample.landmark)}', json_header)
        self.assertIn('left_foot_marker: "left_foot"', app_js)
        self.assertIn("steamVrRequiredLandmarkKeys", app_js)
        self.assertIn("steamVrRequiredComplete(acceptedKeys)", app_js)
        self.assertIn("setActionControl(el.steamVrAlignFinish", app_js)
        self.assertIn("enabled: sessionActive && requiredMet && !busy", app_js)
        self.assertIn("commandPending(\"steamVrAlignmentFinish\")", app_js)
        self.assertIn('data-wiz="chest"', html)
        self.assertIn('data-wiz="left_elbow"', html)

    def test_steamvr_action_buttons_derive_state_from_session_provider_config_and_pending_commands(self) -> None:
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        for required in (
            "renderSteamVrAlignmentButtons(provider, session, acceptedKeys, samplesByKey, requiredMet)",
            "steamVrControllerReady(provider)",
            "landmarkEnabledByConfig(item.key)",
            "pending: recordPending && active",
            "label: done ? `${item.label} ✓` : item.label",
            "updateWizardPrompt(steamVrNextWizardStepForState",
            "renderSteamVrWizardProgress(acceptedKeys)",
        ):
            self.assertIn(required, app_js)

    def test_tracker_space_status_class_distinguishes_stale_from_blocked(self) -> None:
        """F3: 'stale' means degraded output (OSC continues with last numeric
        transform). 'missing'/'idle' means blocked (no output). The status
        class function must not collapse these into the same visual state.
        """
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        status_fn = app_js[app_js.index("function trackerSpaceStatusClass"):app_js.index("function solveStatusClass")]
        # stale/degraded output gets warn class
        self.assertIn('"stale") return "warn"', status_fn)
        # missing/idle blocked output gets bad class
        self.assertIn('"missing" || status === "idle") return "bad"', status_fn)

    def test_body_calibration_complete_no_auto_persist_shows_warn(self) -> None:
        """F7: complete + auto_persist=false must render as warn with distinct
        text, not muted/disabled.
        """
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        self.assertIn("completeNoAutoPersist", app_js)
        self.assertIn("save config manually to persist (auto-persist is off)", app_js)

    def test_depth_postprocess_enabled_visible_when_on(self) -> None:
        """F6: depth_postprocess_enabled has no editable UI control, but when
        true the model box must display it as a visible diagnostic.
        """
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        self.assertIn("depth_postprocess_enabled", app_js)
        self.assertIn("ON (config-only)", app_js)


if __name__ == "__main__":
    unittest.main()
