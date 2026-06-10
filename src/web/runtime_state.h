#pragma once

#include "calibration/calibration_io.h"
#include "core/config.h"
#include "debug/debug_snapshot.h"
#include "inference/rtmpose_session.h"

#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

namespace bt {

struct WebRuntimeBodyOverlayView {
    char slot = 'a';
    bool preview_available = false;
    int preview_width = 0;
    int preview_height = 0;
    int preview_source_x = 0;
    int preview_source_y = 0;
    int preview_source_width = 0;
    int preview_source_height = 0;
    int preview_frame_width = 0;
    int preview_frame_height = 0;
    std::uint64_t preview_sequence = 0;
    std::uint64_t frame_sequence = 0;
    double frame_age_ms = 0.0;
    double stale_timeout_ms = 0.0;
    bool stale = false;
    DecodedPose2D pose{};
    bool pose_available = false;
    bool model_inference_ran = false;
    std::string pose_source = "none";
    std::string freshness = "missing";
    std::string reason = "no runtime pose published";
    float confidence = 0.0f;
};

struct WebRuntimeBodyOverlay {
    WebRuntimeBodyOverlayView camera_a{};
    WebRuntimeBodyOverlayView camera_b{'b'};
};

struct WebRuntimeSnapshot {
    AppConfig config{};
    bool config_loaded = false;
    CalibrationReadiness calibration{};
    bool calibration_loaded = false;
    ModelSessionInfo model{};
    bool model_loaded = false;
    DebugSnapshot debug{};
    std::string camera_a_preview_data_url;
    std::string camera_a_full_preview_data_url;
    int camera_a_preview_width = 0;
    int camera_a_preview_height = 0;
    int camera_a_preview_source_x = 0;
    int camera_a_preview_source_y = 0;
    int camera_a_preview_source_width = 0;
    int camera_a_preview_source_height = 0;
    int camera_a_preview_frame_width = 0;
    int camera_a_preview_frame_height = 0;
    std::uint64_t camera_a_preview_sequence = 0;
    std::string camera_b_preview_data_url;
    std::string camera_b_full_preview_data_url;
    int camera_b_preview_width = 0;
    int camera_b_preview_height = 0;
    int camera_b_preview_source_x = 0;
    int camera_b_preview_source_y = 0;
    int camera_b_preview_source_width = 0;
    int camera_b_preview_source_height = 0;
    int camera_b_preview_frame_width = 0;
    int camera_b_preview_frame_height = 0;
    std::uint64_t camera_b_preview_sequence = 0;
    WebRuntimeBodyOverlay body_overlay{};
};

class WebRuntimeState {
public:
    void SetConfig(const AppConfig& next_config) {
        std::scoped_lock lock(mutex_);
        snapshot_.config = next_config;
        snapshot_.config_loaded = true;
    }

    void SetCalibration(const CalibrationReadiness& readiness) {
        std::scoped_lock lock(mutex_);
        snapshot_.calibration = readiness;
        snapshot_.calibration_loaded = true;
    }

    void SetModel(const ModelSessionInfo& model) {
        std::scoped_lock lock(mutex_);
        snapshot_.model = model;
        snapshot_.model_loaded = model.loaded;
    }

    void SetDebug(const DebugSnapshot& debug) {
        std::scoped_lock lock(mutex_);
        snapshot_.debug = debug;
        UpdateBodyOverlayFromDebugLocked(debug);
    }

    void SetCameraPreview(
        char slot,
        const std::string& data_url,
        const std::string& full_data_url,
        int width,
        int height,
        int source_x,
        int source_y,
        int source_width,
        int source_height,
        int frame_width,
        int frame_height,
        std::uint64_t sequence) {
        std::scoped_lock lock(mutex_);
        if (slot == 'a' || slot == 'A') {
            snapshot_.camera_a_preview_data_url = data_url;
            snapshot_.camera_a_full_preview_data_url = full_data_url;
            snapshot_.camera_a_preview_width = width;
            snapshot_.camera_a_preview_height = height;
            snapshot_.camera_a_preview_source_x = source_x;
            snapshot_.camera_a_preview_source_y = source_y;
            snapshot_.camera_a_preview_source_width = source_width;
            snapshot_.camera_a_preview_source_height = source_height;
            snapshot_.camera_a_preview_frame_width = frame_width;
            snapshot_.camera_a_preview_frame_height = frame_height;
            snapshot_.camera_a_preview_sequence = sequence;
            snapshot_.body_overlay.camera_a.slot = 'a';
            snapshot_.body_overlay.camera_a.preview_available = !data_url.empty();
            snapshot_.body_overlay.camera_a.preview_width = width;
            snapshot_.body_overlay.camera_a.preview_height = height;
            snapshot_.body_overlay.camera_a.preview_source_x = source_x;
            snapshot_.body_overlay.camera_a.preview_source_y = source_y;
            snapshot_.body_overlay.camera_a.preview_source_width = source_width;
            snapshot_.body_overlay.camera_a.preview_source_height = source_height;
            snapshot_.body_overlay.camera_a.preview_frame_width = frame_width;
            snapshot_.body_overlay.camera_a.preview_frame_height = frame_height;
            snapshot_.body_overlay.camera_a.preview_sequence = sequence;
        } else if (slot == 'b' || slot == 'B') {
            snapshot_.camera_b_preview_data_url = data_url;
            snapshot_.camera_b_full_preview_data_url = full_data_url;
            snapshot_.camera_b_preview_width = width;
            snapshot_.camera_b_preview_height = height;
            snapshot_.camera_b_preview_source_x = source_x;
            snapshot_.camera_b_preview_source_y = source_y;
            snapshot_.camera_b_preview_source_width = source_width;
            snapshot_.camera_b_preview_source_height = source_height;
            snapshot_.camera_b_preview_frame_width = frame_width;
            snapshot_.camera_b_preview_frame_height = frame_height;
            snapshot_.camera_b_preview_sequence = sequence;
            snapshot_.body_overlay.camera_b.slot = 'b';
            snapshot_.body_overlay.camera_b.preview_available = !data_url.empty();
            snapshot_.body_overlay.camera_b.preview_width = width;
            snapshot_.body_overlay.camera_b.preview_height = height;
            snapshot_.body_overlay.camera_b.preview_source_x = source_x;
            snapshot_.body_overlay.camera_b.preview_source_y = source_y;
            snapshot_.body_overlay.camera_b.preview_source_width = source_width;
            snapshot_.body_overlay.camera_b.preview_source_height = source_height;
            snapshot_.body_overlay.camera_b.preview_frame_width = frame_width;
            snapshot_.body_overlay.camera_b.preview_frame_height = frame_height;
            snapshot_.body_overlay.camera_b.preview_sequence = sequence;
        }
    }

    void Publish(const DebugSnapshot& debug) {
        std::scoped_lock lock(mutex_);
        // snapshot_.config is already set by SetConfig(); do not
        // re-push a stale public field into the snapshot.
        snapshot_.config_loaded = true;
        snapshot_.debug = debug;
        UpdateBodyOverlayFromDebugLocked(debug);
    }

    WebRuntimeSnapshot Snapshot() const {
        std::scoped_lock lock(mutex_);
        return snapshot_;
    }

private:
    static bool ModelInferenceRanForView(const DecodedPose2D& pose, double inference_ms, double onnx_ms, double decode_ms) {
        return pose.valid &&
            ((std::isfinite(inference_ms) && inference_ms > 0.0) ||
             (std::isfinite(onnx_ms) && onnx_ms > 0.0) ||
             (std::isfinite(decode_ms) && decode_ms > 0.0));
    }

    void UpdateOverlayViewFromDebugLocked(
        WebRuntimeBodyOverlayView& view,
        char slot,
        const DecodedPose2D& pose,
        float pose_confidence,
        std::uint64_t frame_sequence,
        double frame_age_ms,
        double inference_ms,
        double onnx_ms,
        double decode_ms) {
        view.slot = slot;
        view.frame_sequence = frame_sequence;
        view.frame_age_ms = frame_age_ms;
        view.stale_timeout_ms = snapshot_.config.tracking.stale_frame_timeout_ms;
        view.stale = view.stale_timeout_ms > 0.0 &&
            std::isfinite(frame_age_ms) &&
            frame_age_ms > view.stale_timeout_ms;
        view.pose = pose;
        view.pose_available = pose.valid;
        view.model_inference_ran = ModelInferenceRanForView(pose, inference_ms, onnx_ms, decode_ms);
        view.pose_source = view.model_inference_ran ? "model" : (pose.valid ? "runtime_pose_unverified" : "none");
        view.confidence = pose.valid ? pose_confidence : 0.0f;

        if (pose.valid) {
            view.freshness = view.stale ? "stale" : "fresh";
            view.reason = view.model_inference_ran
                ? "runtime model inference decoded this camera pose"
                : "pose present but runtime inference timing was not recorded";
        } else if (view.preview_available) {
            view.freshness = view.stale ? "stale_preview_only" : "preview_only";
            view.reason = "preview frame published but no decoded pose is available";
        } else {
            view.freshness = "missing";
            view.reason = "no preview frame or decoded pose is available";
        }
    }

    void UpdateBodyOverlayFromDebugLocked(const DebugSnapshot& debug) {
        snapshot_.body_overlay.camera_a.preview_available = !snapshot_.camera_a_preview_data_url.empty();
        snapshot_.body_overlay.camera_a.preview_width = snapshot_.camera_a_preview_width;
        snapshot_.body_overlay.camera_a.preview_height = snapshot_.camera_a_preview_height;
        snapshot_.body_overlay.camera_a.preview_source_x = snapshot_.camera_a_preview_source_x;
        snapshot_.body_overlay.camera_a.preview_source_y = snapshot_.camera_a_preview_source_y;
        snapshot_.body_overlay.camera_a.preview_source_width = snapshot_.camera_a_preview_source_width;
        snapshot_.body_overlay.camera_a.preview_source_height = snapshot_.camera_a_preview_source_height;
        snapshot_.body_overlay.camera_a.preview_frame_width = snapshot_.camera_a_preview_frame_width;
        snapshot_.body_overlay.camera_a.preview_frame_height = snapshot_.camera_a_preview_frame_height;
        snapshot_.body_overlay.camera_a.preview_sequence = snapshot_.camera_a_preview_sequence;
        UpdateOverlayViewFromDebugLocked(
            snapshot_.body_overlay.camera_a,
            'a',
            debug.camera_a_pose,
            debug.camera_a_pose_confidence,
            debug.frame_a_sequence,
            debug.camera_a_frame_age_ms,
            debug.inference_ms_a,
            debug.onnx_ms_a,
            debug.decode_ms_a);

        snapshot_.body_overlay.camera_b.preview_available = !snapshot_.camera_b_preview_data_url.empty();
        snapshot_.body_overlay.camera_b.preview_width = snapshot_.camera_b_preview_width;
        snapshot_.body_overlay.camera_b.preview_height = snapshot_.camera_b_preview_height;
        snapshot_.body_overlay.camera_b.preview_source_x = snapshot_.camera_b_preview_source_x;
        snapshot_.body_overlay.camera_b.preview_source_y = snapshot_.camera_b_preview_source_y;
        snapshot_.body_overlay.camera_b.preview_source_width = snapshot_.camera_b_preview_source_width;
        snapshot_.body_overlay.camera_b.preview_source_height = snapshot_.camera_b_preview_source_height;
        snapshot_.body_overlay.camera_b.preview_frame_width = snapshot_.camera_b_preview_frame_width;
        snapshot_.body_overlay.camera_b.preview_frame_height = snapshot_.camera_b_preview_frame_height;
        snapshot_.body_overlay.camera_b.preview_sequence = snapshot_.camera_b_preview_sequence;
        UpdateOverlayViewFromDebugLocked(
            snapshot_.body_overlay.camera_b,
            'b',
            debug.camera_b_pose,
            debug.camera_b_pose_confidence,
            debug.frame_b_sequence,
            debug.camera_b_frame_age_ms,
            debug.inference_ms_b,
            debug.onnx_ms_b,
            debug.decode_ms_b);
    }

    mutable std::mutex mutex_;
    WebRuntimeSnapshot snapshot_{};
};

using RuntimeState = WebRuntimeState;

} // namespace bt
