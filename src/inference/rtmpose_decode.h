#pragma once

#include "core/status.h"
#include "core/types.h"
#include "inference/rtmpose_model_contract.h"
#include "inference/rtmpose_session.h"

#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace bt {

enum class RtmPoseOutputFormat : std::uint8_t {
    Unknown = 0,
    SimCC,
    KeypointsXYConfidence,
    RtmwWholeBody133SimCC,
    Rtmw3dWholeBody133SimCC
};

inline const char* ToString(RtmPoseOutputFormat format) {
    switch (format) {
    case RtmPoseOutputFormat::SimCC: return "simcc";
    case RtmPoseOutputFormat::KeypointsXYConfidence: return "xyc";
    case RtmPoseOutputFormat::RtmwWholeBody133SimCC: return "rtmw_wholebody133_simcc";
    case RtmPoseOutputFormat::Rtmw3dWholeBody133SimCC: return "rtmw3d_wholebody133_simcc";
    default: return "unknown";
    }
}

struct RtmPosePreprocessSpec {
    std::array<float, 3> mean{123.675f, 116.28f, 103.53f};
    std::array<float, 3> std{58.395f, 57.12f, 57.375f};
    float pad_value = 114.0f;
    float simcc_split_ratio = 2.0f;
    bool bgr_to_rgb = true;
};

struct ImagePreprocessMeta {
    int source_image_width = 0;
    int source_image_height = 0;
    Rect2f source_region{};
    int model_input_width = 0;
    int model_input_height = 0;
    float resize_scale = 1.0f;
    float pad_left = 0.0f;
    float pad_top = 0.0f;
};

struct DecodedPose2D {
    bool valid = false;
    RtmPoseOutputFormat format = RtmPoseOutputFormat::Unknown;
    KeypointArray keypoints{};
    float aggregate_confidence = 0.0f;
};

// Per-keypoint depth decoded from the RTMW3D z-SimCC tensor (outputs[2]).
// This is model/body-relative depth — NOT metric world z.
// It must never be labelled or used as world-space z without an explicit named transform.
struct ModelDepthKeypoint {
    float raw_z_bin     = 0.0f;  // argmax bin index in the z-SimCC axis
    float refined_z     = 0.0f;  // parabolic-refined sub-bin index
    float z_logit       = 0.0f;  // peak logit from the z-SimCC distribution
    float confidence_3d = 0.0f;  // geometric mean of x/y/z SimCC confidences
    bool  z_decoded     = false; // true only when outputs[2] was actually consumed
};

// Companion to DecodedPose2D.  Valid only when the model is RTMW3D (3-tensor SimCC).
// coordinate_frame is always "model_simcc_body_relative".
// model_depth[i] corresponds to DecodedPose2D::keypoints[i] (Halpe-26 indexing).
struct DecodedPose3D {
    bool valid = false;
    std::string coordinate_frame = "model_simcc_body_relative";
    std::array<ModelDepthKeypoint, kHalpe26Count> model_depth{};
};

// Both decode results bundled: pose2d always set on success, pose3d.valid only for RTMW3D.
struct DecodedPoseWithDepth {
    DecodedPose2D pose2d;
    DecodedPose3D pose3d;
};

struct RtmPoseInputPacket {
    TensorF32 tensor;
    ImagePreprocessMeta meta;
};

Status FillRtmPoseInputPacket(
    const cv::Mat& bgr_image,
    const Rect2f& source_region,
    const ModelSessionInfo& info,
    RtmPoseInputPacket& packet,
    const RtmPosePreprocessSpec& spec = {});
Result<RtmPoseInputPacket> PreprocessSourceRegionToRtmPoseInput(
    const cv::Mat& bgr_image,
    const Rect2f& source_region,
    const ModelSessionInfo& info,
    const RtmPosePreprocessSpec& spec = {});
Result<RtmPoseInputPacket> PreprocessWholeImageToRtmPoseInput(
    const cv::Mat& bgr_image,
    const ModelSessionInfo& info,
    const RtmPosePreprocessSpec& spec = {});
Vec2f ModelPointToSourceImage(const ImagePreprocessMeta& meta, float model_x, float model_y);

// Legacy 2D-only decode.  Calls DecodeRtmPoseOutputsWithDepth internally; z is discarded.
// Existing callers are unchanged; the live path is being migrated to DecodeRtmPoseOutputsWithDepth.
Result<DecodedPose2D> DecodeRtmPoseOutputs(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec = {});

// Full decode: returns 2D pose + model-depth companion when the model is RTMW3D.
// For non-3D models, pose3d.valid is false and pose2d is still populated.
// The z values in pose3d carry coordinate_frame "model_simcc_body_relative" and
// are never metric world coordinates.
Result<DecodedPoseWithDepth> DecodeRtmPoseOutputsWithDepth(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec = {});

std::string BuildPreprocessSummary(const ImagePreprocessMeta& meta);
std::string BuildDecodedPoseSummary(const DecodedPose2D& pose);

} // namespace bt
