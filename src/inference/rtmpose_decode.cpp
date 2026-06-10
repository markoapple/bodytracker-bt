#include "inference/rtmpose_decode.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <sstream>

namespace bt {
namespace {

constexpr std::size_t kRtmw3dWholeBodyKeypointCountSize =
    static_cast<std::size_t>(kRtmw3dWholeBodyKeypointCount);

bool IsFinite(float v) {
    return std::isfinite(v);
}

Vec2f InvalidPoint() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return Vec2f{nan, nan};
}

Status ValidatePreprocessSpec(const RtmPosePreprocessSpec& spec) {
    if (!std::isfinite(spec.simcc_split_ratio) || spec.simcc_split_ratio <= 0.0f) {
        return Status::Error(StatusCode::ValidationError, "RTMPose SimCC split ratio must be positive and finite");
    }
    if (!std::isfinite(spec.pad_value)) {
        return Status::Error(StatusCode::ValidationError, "RTMPose pad value must be finite");
    }
    for (std::size_t i = 0; i < spec.mean.size(); ++i) {
        if (!std::isfinite(spec.mean[i]) || !std::isfinite(spec.std[i]) || spec.std[i] == 0.0f) {
            return Status::Error(StatusCode::ValidationError, "RTMPose preprocessing mean/std values must be finite and std must be non-zero");
        }
    }
    return Status::OK();
}


Status ValidatePreprocessMeta(const ImagePreprocessMeta& meta) {
    if (meta.source_image_width <= 0 || meta.source_image_height <= 0 ||
        meta.model_input_width <= 0 || meta.model_input_height <= 0) {
        return Status::Error(StatusCode::ValidationError, "RTMPose preprocess metadata image dimensions must be positive");
    }
    if (!(meta.resize_scale > 0.0f) ||
        !std::isfinite(meta.resize_scale) ||
        !std::isfinite(meta.pad_left) ||
        !std::isfinite(meta.pad_top) ||
        !std::isfinite(meta.source_region.x) ||
        !std::isfinite(meta.source_region.y) ||
        !std::isfinite(meta.source_region.width) ||
        !std::isfinite(meta.source_region.height) ||
        !(meta.source_region.width > 0.0f) ||
        !(meta.source_region.height > 0.0f)) {
        return Status::Error(StatusCode::ValidationError, "RTMPose preprocess metadata contains invalid source region, scale, or padding");
    }
    return Status::OK();
}

void StoreDecodedKeypoint(Keypoint2D& kp, const Vec2f& pixel, float confidence) {
    kp.pixel = pixel;
    kp.confidence = std::isfinite(confidence) ? std::max(0.0f, std::min(1.0f, confidence)) : 0.0f;
    kp.present = std::isfinite(pixel.x) && std::isfinite(pixel.y) && kp.confidence > 0.0f;
}

float ConfidenceLikeTo01(float v) {
    if (!IsFinite(v)) {
        return 0.0f;
    }
    if (v >= 0.0f && v <= 1.0f) {
        return v;
    }
    const float clamped = std::max(-20.0f, std::min(20.0f, v));
    return 1.0f / (1.0f + std::exp(-clamped));
}

bool LooksLikeSimCCOutput(const std::vector<NamedTensorF32>& outputs) {
    if (outputs.size() != 2) {
        return false;
    }

    const auto& a = outputs[0].tensor.shape;
    const auto& b = outputs[1].tensor.shape;
    return a.size() == 3 &&
        b.size() == 3 &&
        a[0] == 1 &&
        b[0] == 1 &&
        a[1] == b[1] &&
        a[1] > 0 &&
        a[2] > 0 &&
        b[2] > 0;
}

bool LooksLikeRtmwWholeBodyOutput(const std::vector<NamedTensorF32>& outputs) {
    if (outputs.size() != 2) {
        return false;
    }
    const auto& x = outputs[0].tensor.shape;
    const auto& y = outputs[1].tensor.shape;
    return x.size() == 3 &&
        y.size() == 3 &&
        x[0] == 1 &&
        y[0] == 1 &&
        x[1] == kRtmw3dWholeBodyKeypointCount &&
        y[1] == kRtmw3dWholeBodyKeypointCount &&
        x[2] == kRtmPoseHalpe26SimccXBins &&
        y[2] == kRtmPoseHalpe26SimccYBins;
}

bool LooksLikeXYCOutput(const std::vector<NamedTensorF32>& outputs) {
    if (outputs.size() != 1) {
        return false;
    }
    const auto& s = outputs[0].tensor.shape;
    if (s.size() == 3) {
        return s[0] == 1 && s[1] > 0 && s[2] >= 3;
    }
    if (s.size() == 2) {
        return s[0] > 0 && s[1] >= 3;
    }
    return false;
}

bool LooksLikeRtmw3dWholeBodyOutput(const std::vector<NamedTensorF32>& outputs) {
    if (outputs.size() != 3) {
        return false;
    }
    const auto& x = outputs[0].tensor.shape;
    const auto& y = outputs[1].tensor.shape;
    const auto& z = outputs[2].tensor.shape;
    return x.size() == 3 &&
        y.size() == 3 &&
        z.size() == 3 &&
        x[0] == 1 &&
        y[0] == 1 &&
        z[0] == 1 &&
        x[1] == kRtmw3dWholeBodyKeypointCount &&
        y[1] == kRtmw3dWholeBodyKeypointCount &&
        z[1] == kRtmw3dWholeBodyKeypointCount &&
        x[2] == kRtmPoseHalpe26SimccXBins &&
        y[2] == kRtmPoseHalpe26SimccYBins &&
        z[2] == kRtmw3dSimccZBins;
}

std::size_t TensorElementCountFromShape(const std::vector<std::int64_t>& shape) {
    std::size_t count = 1;
    for (const auto dim : shape) {
        if (dim <= 0) {
            return 0;
        }
        count *= static_cast<std::size_t>(dim);
    }
    return count;
}

Status ValidateTensorBuffer(const NamedTensorF32& tensor) {
    const std::size_t expected = TensorElementCountFromShape(tensor.tensor.shape);
    if (expected == 0) {
        return Status::Error(StatusCode::ValidationError, "Tensor has non-positive shape dimension: " + tensor.name);
    }
    if (tensor.tensor.data.size() != expected) {
        std::ostringstream oss;
        oss << "Tensor data size mismatch for \"" << tensor.name
            << "\": shape expects " << expected
            << " elements, buffer has " << tensor.tensor.data.size();
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    return Status::OK();
}

std::size_t FindArgMax(const float* data, std::size_t count, float& max_value) {
    max_value = -std::numeric_limits<float>::infinity();
    std::size_t max_idx = 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (data[i] > max_value) {
            max_value = data[i];
            max_idx = i;
        }
    }
    return max_idx;
}

float RefineSimccPeakIndex(const float* data, std::size_t count, std::size_t peak_idx) {
    if (!data || count == 0 || peak_idx >= count) {
        return 0.0f;
    }
    if (peak_idx == 0 || peak_idx + 1 >= count) {
        return static_cast<float>(peak_idx);
    }

    const float left = data[peak_idx - 1];
    const float center = data[peak_idx];
    const float right = data[peak_idx + 1];
    if (!std::isfinite(left) || !std::isfinite(center) || !std::isfinite(right)) {
        return static_cast<float>(peak_idx);
    }

    // SimCC decoders must not stop at the integer argmax bin. Fit a local
    // parabola to the peak and its immediate neighbours, which works for raw
    // logits as well as probability-like SimCC vectors. The argmax remains the
    // authority; refinement is clamped to half a bin so noisy shoulders cannot
    // jump into the neighbouring bin.
    const float denom = left - 2.0f * center + right;
    if (!std::isfinite(denom) || std::abs(denom) <= 1.0e-6f) {
        return static_cast<float>(peak_idx);
    }
    const float offset = 0.5f * (left - right) / denom;
    if (!std::isfinite(offset)) {
        return static_cast<float>(peak_idx);
    }
    const float clamped_offset = std::max(-0.5f, std::min(0.5f, offset));
    return static_cast<float>(peak_idx) + clamped_offset;
}

Result<Rect2f> ClampPositiveSourceRegion(const Rect2f& r, int image_w, int image_h) {
    if (image_w <= 0 || image_h <= 0) {
        return Status::Error(StatusCode::ValidationError, "Source image dimensions must be positive");
    }
    if (!std::isfinite(r.x) ||
        !std::isfinite(r.y) ||
        !std::isfinite(r.width) ||
        !std::isfinite(r.height) ||
        !(r.width > 0.0f) ||
        !(r.height > 0.0f)) {
        return Status::Error(StatusCode::ValidationError, "RTMPose source ROI must contain finite positive dimensions");
    }
    Rect2f out = r;
    out.x = std::max(-out.width + 1.0f, std::min(static_cast<float>(image_w - 1), out.x));
    out.y = std::max(-out.height + 1.0f, std::min(static_cast<float>(image_h - 1), out.y));
    return out;
}

Result<DecodedPose2D> DecodeSimCC(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec) {

    const auto& s0 = outputs[0].tensor.shape;
    const auto& s1 = outputs[1].tensor.shape;
    if (const auto s = ValidateTensorBuffer(outputs[0]); !s.ok()) {
        return s;
    }
    if (const auto s = ValidateTensorBuffer(outputs[1]); !s.ok()) {
        return s;
    }

    const std::size_t joint_count = static_cast<std::size_t>(s0[1]);

    if (joint_count != kHalpe26Count) {
        std::ostringstream oss;
        oss << "SimCC decode expects " << kHalpe26Count
            << " joints for Halpe26, model outputs " << joint_count;
        return Status::Error(StatusCode::ValidationError, oss.str());
    }

    const std::size_t len0 = static_cast<std::size_t>(s0[2]);
    const std::size_t len1 = static_cast<std::size_t>(s1[2]);
    const float expected_x = static_cast<float>(meta.model_input_width) * spec.simcc_split_ratio;
    const float expected_y = static_cast<float>(meta.model_input_height) * spec.simcc_split_ratio;
    const float score_as_xy = std::abs(static_cast<float>(len0) - expected_x) + std::abs(static_cast<float>(len1) - expected_y);
    const float score_as_yx = std::abs(static_cast<float>(len0) - expected_y) + std::abs(static_cast<float>(len1) - expected_x);

    const NamedTensorF32* x_tensor = &outputs[0];
    const NamedTensorF32* y_tensor = &outputs[1];
    std::size_t x_len = len0;
    std::size_t y_len = len1;
    if (score_as_yx < score_as_xy) {
        x_tensor = &outputs[1];
        y_tensor = &outputs[0];
        x_len = len1;
        y_len = len0;
    }

    DecodedPose2D decoded;
    decoded.valid = true;
    decoded.format = RtmPoseOutputFormat::SimCC;

    float confidence_sum = 0.0f;
    for (std::size_t j = 0; j < joint_count; ++j) {
        const float* x_data = x_tensor->tensor.data.data() + (j * x_len);
        const float* y_data = y_tensor->tensor.data.data() + (j * y_len);

        float max_x = 0.0f;
        float max_y = 0.0f;
        const std::size_t x_idx = FindArgMax(x_data, x_len, max_x);
        const std::size_t y_idx = FindArgMax(y_data, y_len, max_y);

        const float model_x = RefineSimccPeakIndex(x_data, x_len, x_idx) / spec.simcc_split_ratio;
        const float model_y = RefineSimccPeakIndex(y_data, y_len, y_idx) / spec.simcc_split_ratio;
        const float conf = std::sqrt(ConfidenceLikeTo01(max_x) * ConfidenceLikeTo01(max_y));

        auto& kp = decoded.keypoints[j];
        StoreDecodedKeypoint(kp, ModelPointToSourceImage(meta, model_x, model_y), conf);
        confidence_sum += kp.confidence;
    }

    decoded.aggregate_confidence = confidence_sum / static_cast<float>(joint_count);
    return decoded;
}

void CopyWholeBodyKeypoint(
    DecodedPose2D& decoded,
    KeypointId target,
    const std::array<Keypoint2D, kRtmw3dWholeBodyKeypointCountSize>& source,
    std::size_t source_index) {
    decoded.keypoints[static_cast<std::size_t>(target)] = source[source_index];
}

void StoreMidpointKeypoint(
    DecodedPose2D& decoded,
    KeypointId target,
    const Keypoint2D& a,
    const Keypoint2D& b,
    float confidence_scale = 1.0f) {
    auto& out = decoded.keypoints[static_cast<std::size_t>(target)];
    if (!a.present || !b.present) {
        out.present = false;
        out.confidence = 0.0f;
        out.pixel = InvalidPoint();
        return;
    }
    const float confidence = std::max(0.0f, std::min(1.0f, 0.5f * (a.confidence + b.confidence) * confidence_scale));
    StoreDecodedKeypoint(
        out,
        Vec2f{0.5f * (a.pixel.x + b.pixel.x), 0.5f * (a.pixel.y + b.pixel.y)},
        confidence);
}

DecodedPose2D MapWholeBody133ToInternal26(
    const std::array<Keypoint2D, kRtmw3dWholeBodyKeypointCountSize>& wholebody,
    RtmPoseOutputFormat format) {
    DecodedPose2D decoded;
    decoded.valid = true;
    decoded.format = format;

    CopyWholeBodyKeypoint(decoded, KeypointId::Nose, wholebody, 0);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftEye, wholebody, 1);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightEye, wholebody, 2);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftEar, wholebody, 3);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightEar, wholebody, 4);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftShoulder, wholebody, 5);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightShoulder, wholebody, 6);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftElbow, wholebody, 7);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightElbow, wholebody, 8);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftWrist, wholebody, 9);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightWrist, wholebody, 10);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftHip, wholebody, 11);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightHip, wholebody, 12);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftKnee, wholebody, 13);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightKnee, wholebody, 14);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftAnkle, wholebody, 15);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightAnkle, wholebody, 16);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftBigToe, wholebody, 17);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftSmallToe, wholebody, 18);
    CopyWholeBodyKeypoint(decoded, KeypointId::LeftHeel, wholebody, 19);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightBigToe, wholebody, 20);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightSmallToe, wholebody, 21);
    CopyWholeBodyKeypoint(decoded, KeypointId::RightHeel, wholebody, 22);
    StoreMidpointKeypoint(decoded, KeypointId::Neck, wholebody[5], wholebody[6]);
    StoreMidpointKeypoint(decoded, KeypointId::Pelvis, wholebody[11], wholebody[12]);
    decoded.keypoints[static_cast<std::size_t>(KeypointId::HeadTop)] = wholebody[0];
    decoded.keypoints[static_cast<std::size_t>(KeypointId::HeadTop)].confidence *= 0.75f;

    float confidence_sum = 0.0f;
    for (const auto& kp : decoded.keypoints) {
        confidence_sum += kp.confidence;
    }
    decoded.aggregate_confidence = confidence_sum / static_cast<float>(decoded.keypoints.size());
    return decoded;
}

Result<DecodedPose2D> DecodeRtmwWholeBodySimCC(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec) {

    for (const auto& output : outputs) {
        if (const auto s = ValidateTensorBuffer(output); !s.ok()) {
            return s;
        }
    }

    const auto& x_shape = outputs[0].tensor.shape;
    const auto& y_shape = outputs[1].tensor.shape;
    const std::size_t joint_count = static_cast<std::size_t>(x_shape[1]);
    const std::size_t x_len = static_cast<std::size_t>(x_shape[2]);
    const std::size_t y_len = static_cast<std::size_t>(y_shape[2]);

    if (joint_count != static_cast<std::size_t>(kRtmw3dWholeBodyKeypointCount)) {
        std::ostringstream oss;
        oss << "RTMW whole-body decode expects " << kRtmw3dWholeBodyKeypointCount
            << " keypoints, model outputs " << joint_count;
        return Status::Error(StatusCode::ValidationError, oss.str());
    }

    std::array<Keypoint2D, kRtmw3dWholeBodyKeypointCountSize> wholebody{};
    for (std::size_t j = 0; j < joint_count; ++j) {
        const float* x_data = outputs[0].tensor.data.data() + (j * x_len);
        const float* y_data = outputs[1].tensor.data.data() + (j * y_len);

        float max_x = 0.0f;
        float max_y = 0.0f;
        const std::size_t x_idx = FindArgMax(x_data, x_len, max_x);
        const std::size_t y_idx = FindArgMax(y_data, y_len, max_y);

        const float model_x = RefineSimccPeakIndex(x_data, x_len, x_idx) / spec.simcc_split_ratio;
        const float model_y = RefineSimccPeakIndex(y_data, y_len, y_idx) / spec.simcc_split_ratio;
        const float conf = std::sqrt(ConfidenceLikeTo01(max_x) * ConfidenceLikeTo01(max_y));
        StoreDecodedKeypoint(wholebody[j], ModelPointToSourceImage(meta, model_x, model_y), conf);
    }

    return MapWholeBody133ToInternal26(wholebody, RtmPoseOutputFormat::RtmwWholeBody133SimCC);
}

// Mapping: wholebody-133 source index -> Halpe-26 destination index.
// kHalpe26Count means no direct mapping (midpoint or derived joints handled separately).
// Indices 0-22 cover the joints that have a direct 1:1 mapping.
//
// Cocktail14/WholeBody-133 keypoint ordering (COCO-WholeBody standard):
//   0-16: body keypoints (nose, eyes, ears, shoulders, elbows, wrists,
//         left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle)
//   17-22: left_foot (big_toe=17, small_toe=18, heel=19),
//          right_foot (big_toe=20, small_toe=21, heel=22)
//   23-132: face keypoints (68) + left_hand (21) + right_hand (21)
//
// Internal keypoint ordering (see kInternalKeypointOrder):
//   0-16: same body keypoints (identical mapping for 0-16)
//   17: head_top (derived from nose at reduced confidence)
//   18: neck (midpoint of shoulders 5+6)
//   19: pelvis (midpoint of hips 11+12)
//   20-25: left_big_toe, right_big_toe, left_small_toe, right_small_toe,
//          left_heel, right_heel
//
// The direct mapping reorders indices 17-22 from WholeBody foot order to
// Halpe-26 foot order, then synthesizes 17-19 via midpoints.
static std::size_t WholeBo133ToHalpe26Direct(std::size_t src) {
    // direct keypoint mapping: wholebody-133 index -> halpe-26 index
    static const std::size_t kMap[] = {
        0,1,2,3,4,  // nose,eyes,ears
        5,6,7,8,9,10, // shoulders,elbows,wrists
        11,12,         // hips
        13,14,         // knees
        15,16,         // ankles
        20,22,24,      // left foot (big toe, small toe, heel)
        21,23,25       // right foot
    };
    return src < sizeof(kMap)/sizeof(kMap[0]) ? kMap[src] : kHalpe26Count;
}

// Map 133-element wholebody depth array to Halpe-26, handling midpoint joints.
static std::array<ModelDepthKeypoint, kHalpe26Count> MapWholebody133DepthToHalpe26(
    const std::array<ModelDepthKeypoint, kRtmw3dWholeBodyKeypointCountSize>& wb) {

    std::array<ModelDepthKeypoint, kHalpe26Count> out{};
    for (std::size_t s = 0; s < 23 && s < kRtmw3dWholeBodyKeypointCountSize; ++s) {
        const std::size_t d = WholeBo133ToHalpe26Direct(s);
        if (d < kHalpe26Count) { out[d] = wb[s]; }
    }
    // Neck = midpoint shoulder 5+6
    auto mid = [&](std::size_t dst, std::size_t a, std::size_t b) {
        out[dst].z_decoded     = wb[a].z_decoded && wb[b].z_decoded;
        out[dst].raw_z_bin     = 0.5f * (wb[a].raw_z_bin + wb[b].raw_z_bin);
        out[dst].refined_z     = 0.5f * (wb[a].refined_z + wb[b].refined_z);
        out[dst].z_logit       = 0.5f * (wb[a].z_logit + wb[b].z_logit);
        out[dst].confidence_3d = 0.5f * (wb[a].confidence_3d + wb[b].confidence_3d);
    };
    mid(static_cast<std::size_t>(KeypointId::Neck),   5, 6);
    mid(static_cast<std::size_t>(KeypointId::Pelvis), 11, 12);
    // HeadTop uses nose (wb[0]) at reduced confidence
    out[static_cast<std::size_t>(KeypointId::HeadTop)] = wb[0];
    out[static_cast<std::size_t>(KeypointId::HeadTop)].confidence_3d *= 0.75f;
    return out;
}

// Internal: decode all three SimCC tensors (x, y, z) for RTMW3D.
// Returns both the 2D keypoints and the model-depth companion.
// z values are body-relative SimCC bins, NOT metric world coordinates.
static Result<DecodedPoseWithDepth> DecodeRtmw3dWholeBodySimCCFull(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec) {

    for (const auto& output : outputs) {
        if (const auto s = ValidateTensorBuffer(output); !s.ok()) { return s; }
    }

    const std::size_t joint_count = static_cast<std::size_t>(outputs[0].tensor.shape[1]);
    const std::size_t x_len       = static_cast<std::size_t>(outputs[0].tensor.shape[2]);
    const std::size_t y_len       = static_cast<std::size_t>(outputs[1].tensor.shape[2]);
    const std::size_t z_len       = static_cast<std::size_t>(outputs[2].tensor.shape[2]);

    if (joint_count != static_cast<std::size_t>(kRtmw3dWholeBodyKeypointCount)) {
        std::ostringstream oss;
        oss << "RTMW3D decode expects " << kRtmw3dWholeBodyKeypointCount
            << " keypoints, got " << joint_count;
        return Status::Error(StatusCode::ValidationError, oss.str());
    }

    std::array<Keypoint2D,         kRtmw3dWholeBodyKeypointCountSize> wholebody{};
    std::array<ModelDepthKeypoint, kRtmw3dWholeBodyKeypointCountSize> wholebody_depth{};

    for (std::size_t j = 0; j < joint_count; ++j) {
        const float* xd = outputs[0].tensor.data.data() + (j * x_len);
        const float* yd = outputs[1].tensor.data.data() + (j * y_len);
        const float* zd = outputs[2].tensor.data.data() + (j * z_len);

        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        const std::size_t xi = FindArgMax(xd, x_len, mx);
        const std::size_t yi = FindArgMax(yd, y_len, my);
        const std::size_t zi = FindArgMax(zd, z_len, mz);

        const float model_x   = RefineSimccPeakIndex(xd, x_len, xi) / spec.simcc_split_ratio;
        const float model_y   = RefineSimccPeakIndex(yd, y_len, yi) / spec.simcc_split_ratio;
        const float refined_z = RefineSimccPeakIndex(zd, z_len, zi);  // kept as bin index, not /split_ratio

        const float conf_xy = std::sqrt(ConfidenceLikeTo01(mx) * ConfidenceLikeTo01(my));
        const float conf_3d = std::cbrt(ConfidenceLikeTo01(mx) * ConfidenceLikeTo01(my) * ConfidenceLikeTo01(mz));

        StoreDecodedKeypoint(wholebody[j], ModelPointToSourceImage(meta, model_x, model_y), conf_xy);

        wholebody_depth[j].raw_z_bin     = static_cast<float>(zi);
        wholebody_depth[j].refined_z     = refined_z;
        wholebody_depth[j].z_logit       = mz;
        wholebody_depth[j].confidence_3d = conf_3d;
        wholebody_depth[j].z_decoded     = true;
    }

    DecodedPoseWithDepth result;
    result.pose2d               = MapWholeBody133ToInternal26(wholebody, RtmPoseOutputFormat::Rtmw3dWholeBody133SimCC);
    result.pose3d.valid          = true;
    result.pose3d.coordinate_frame = "model_simcc_body_relative";
    result.pose3d.model_depth    = MapWholebody133DepthToHalpe26(wholebody_depth);
    return result;
}

// Backward-compatible 2D-only wrapper (z is decoded internally and discarded).
Result<DecodedPose2D> DecodeRtmw3dWholeBodySimCC(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec) {

    auto r = DecodeRtmw3dWholeBodySimCCFull(outputs, meta, spec);
    if (!r.ok()) { return r.status(); }
    return r.value().pose2d;
}

Result<DecodedPose2D> DecodeXYC(const std::vector<NamedTensorF32>& outputs, const ImagePreprocessMeta& meta) {
    const auto& tensor = outputs[0].tensor;
    const auto& shape = tensor.shape;
    if (const auto s = ValidateTensorBuffer(outputs[0]); !s.ok()) {
        return s;
    }

    std::size_t joint_count = 0;
    std::size_t stride = 0;
    if (shape.size() == 3) {
        joint_count = static_cast<std::size_t>(shape[1]);
        stride = static_cast<std::size_t>(shape[2]);
    } else if (shape.size() == 2) {
        joint_count = static_cast<std::size_t>(shape[0]);
        stride = static_cast<std::size_t>(shape[1]);
    }

    if (joint_count != kHalpe26Count) {
        std::ostringstream oss;
        oss << "XYC decode expects " << kHalpe26Count
            << " joints for Halpe26, model outputs " << joint_count;
        return Status::Error(StatusCode::ValidationError, oss.str());
    }

    DecodedPose2D decoded;
    decoded.valid = true;
    decoded.format = RtmPoseOutputFormat::KeypointsXYConfidence;

    bool normalized = true;
    for (std::size_t j = 0; j < joint_count; ++j) {
        const float x = tensor.data[j * stride + 0];
        const float y = tensor.data[j * stride + 1];
        if (!IsFinite(x) || !IsFinite(y)) {
            return Status::Error(StatusCode::ValidationError, "XYC output contains non-finite coordinates");
        }
        if (std::abs(x) > 2.0f || std::abs(y) > 2.0f) {
            normalized = false;
            break;
        }
    }

    float confidence_sum = 0.0f;
    for (std::size_t j = 0; j < joint_count; ++j) {
        float model_x = tensor.data[j * stride + 0];
        float model_y = tensor.data[j * stride + 1];
        if (normalized) {
            model_x *= static_cast<float>(meta.model_input_width - 1);
            model_y *= static_cast<float>(meta.model_input_height - 1);
        }
        const float conf = ConfidenceLikeTo01(tensor.data[j * stride + 2]);
        auto& kp = decoded.keypoints[j];
        StoreDecodedKeypoint(kp, ModelPointToSourceImage(meta, model_x, model_y), conf);
        confidence_sum += kp.confidence;
    }

    decoded.aggregate_confidence = confidence_sum / static_cast<float>(joint_count);
    return decoded;
}

} // namespace


Status FillRtmPoseInputPacket(
    const cv::Mat& bgr_image,
    const Rect2f& source_region,
    const ModelSessionInfo& info,
    RtmPoseInputPacket& packet,
    const RtmPosePreprocessSpec& spec) {

    if (const auto s = ValidateRtmPoseImageInputContract(info); !s.ok()) {
        return s;
    }
    if (const auto s = ValidatePreprocessSpec(spec); !s.ok()) {
        return s;
    }
    if (bgr_image.empty()) {
        return Status::Error(StatusCode::InvalidArgument, "Source image is empty");
    }
    if (bgr_image.channels() != 3) {
        return Status::Error(StatusCode::Unsupported, "RTMPose preprocessing requires a 3-channel BGR image");
    }

    const int src_w = bgr_image.cols;
    const int src_h = bgr_image.rows;
    const auto region_result = ClampPositiveSourceRegion(source_region, src_w, src_h);
    if (!region_result.ok()) {
        return region_result.status();
    }
    const Rect2f region = region_result.value();
    const auto& input = info.inputs.front();
    const int model_h = static_cast<int>(input.dims[2]);
    const int model_w = static_cast<int>(input.dims[3]);

    const float scale = std::min(
        static_cast<float>(model_w) / region.width,
        static_cast<float>(model_h) / region.height);
    if (!(scale > 0.0f) || !std::isfinite(scale)) {
        return Status::Error(StatusCode::ValidationError, "Invalid RTMPose ROI resize scale");
    }

    const int resized_w = std::max(1, static_cast<int>(std::lround(region.width * scale)));
    const int resized_h = std::max(1, static_cast<int>(std::lround(region.height * scale)));
    const int pad_left = std::max(0, static_cast<int>(std::floor(0.5f * static_cast<float>(model_w - resized_w))));
    const int pad_top = std::max(0, static_cast<int>(std::floor(0.5f * static_cast<float>(model_h - resized_h))));

    cv::Mat resized_crop;
    cv::Mat warp_m = (cv::Mat_<double>(2, 3) <<
        1.0 / static_cast<double>(scale), 0.0, static_cast<double>(region.x),
        0.0, 1.0 / static_cast<double>(scale), static_cast<double>(region.y));

    cv::warpAffine(
        bgr_image,
        resized_crop,
        warp_m,
        cv::Size(resized_w, resized_h),
        cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
        cv::BORDER_CONSTANT,
        cv::Scalar(spec.pad_value, spec.pad_value, spec.pad_value));

    cv::Mat canvas(model_h, model_w, CV_8UC3, cv::Scalar(spec.pad_value, spec.pad_value, spec.pad_value));
    const int copy_w = std::min(resized_w, model_w - pad_left);
    const int copy_h = std::min(resized_h, model_h - pad_top);
    if (copy_w <= 0 || copy_h <= 0) {
        return Status::Error(StatusCode::InternalError, "Invalid RTMPose copy region");
    }
    resized_crop(cv::Rect(0, 0, copy_w, copy_h)).copyTo(canvas(cv::Rect(pad_left, pad_top, copy_w, copy_h)));

    packet.meta.source_image_width = src_w;
    packet.meta.source_image_height = src_h;
    packet.meta.source_region = region;
    packet.meta.model_input_width = model_w;
    packet.meta.model_input_height = model_h;
    packet.meta.resize_scale = scale;
    packet.meta.pad_left = static_cast<float>(pad_left);
    packet.meta.pad_top = static_cast<float>(pad_top);

    packet.tensor.shape = {1, 3, model_h, model_w};
    packet.tensor.data.resize(static_cast<std::size_t>(3) * static_cast<std::size_t>(model_h) * static_cast<std::size_t>(model_w));
    const std::size_t channel_stride = static_cast<std::size_t>(model_h) * static_cast<std::size_t>(model_w);
    for (int y = 0; y < model_h; ++y) {
        const auto* row = canvas.ptr<unsigned char>(y);
        for (int x = 0; x < model_w; ++x) {
            const int src = 3 * x;
            const float pix[3] = {
                static_cast<float>(row[src + (spec.bgr_to_rgb ? 2 : 0)]),
                static_cast<float>(row[src + 1]),
                static_cast<float>(row[src + (spec.bgr_to_rgb ? 0 : 2)])
            };
            for (int c = 0; c < 3; ++c) {
                const std::size_t dst =
                    static_cast<std::size_t>(c) * channel_stride +
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(model_w) +
                    static_cast<std::size_t>(x);
                packet.tensor.data[dst] = (pix[c] - spec.mean[static_cast<std::size_t>(c)]) / spec.std[static_cast<std::size_t>(c)];
            }
        }
    }

    return Status::OK();
}

Result<RtmPoseInputPacket> PreprocessSourceRegionToRtmPoseInput(
    const cv::Mat& bgr_image,
    const Rect2f& source_region,
    const ModelSessionInfo& info,
    const RtmPosePreprocessSpec& spec) {

    RtmPoseInputPacket packet;
    if (const auto s = FillRtmPoseInputPacket(bgr_image, source_region, info, packet, spec); !s.ok()) {
        return s;
    }
    return packet;
}

Result<RtmPoseInputPacket> PreprocessWholeImageToRtmPoseInput(
    const cv::Mat& bgr_image,
    const ModelSessionInfo& info,
    const RtmPosePreprocessSpec& spec) {
    if (bgr_image.empty()) {
        return Status::Error(StatusCode::InvalidArgument, "Source image is empty");
    }
    return PreprocessSourceRegionToRtmPoseInput(
        bgr_image,
        Rect2f{0.0f, 0.0f, static_cast<float>(bgr_image.cols), static_cast<float>(bgr_image.rows)},
        info,
        spec);
}

Vec2f ModelPointToSourceImage(const ImagePreprocessMeta& meta, float model_x, float model_y) {
    if (!(meta.resize_scale > 0.0f) ||
        !std::isfinite(meta.resize_scale) ||
        !std::isfinite(model_x) ||
        !std::isfinite(model_y) ||
        !std::isfinite(meta.pad_left) ||
        !std::isfinite(meta.pad_top) ||
        !std::isfinite(meta.source_region.x) ||
        !std::isfinite(meta.source_region.y)) {
        return InvalidPoint();
    }
    return Vec2f{
        ((model_x - meta.pad_left) / meta.resize_scale) + meta.source_region.x,
        ((model_y - meta.pad_top) / meta.resize_scale) + meta.source_region.y
    };
}

Result<DecodedPose2D> DecodeRtmPoseOutputs(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec) {

    if (outputs.empty()) {
        return Status::Error(StatusCode::InvalidArgument, "DecodeRtmPoseOutputs received zero outputs");
    }
    if (const auto s = ValidatePreprocessMeta(meta); !s.ok()) {
        return s;
    }
    if (const auto s = ValidatePreprocessSpec(spec); !s.ok()) {
        return s;
    }
    if (LooksLikeRtmw3dWholeBodyOutput(outputs)) {
        return DecodeRtmw3dWholeBodySimCC(outputs, meta, spec);
    }
    if (LooksLikeRtmwWholeBodyOutput(outputs)) {
        return DecodeRtmwWholeBodySimCC(outputs, meta, spec);
    }
    if (LooksLikeSimCCOutput(outputs)) {
        return DecodeSimCC(outputs, meta, spec);
    }
    if (LooksLikeXYCOutput(outputs)) {
        return DecodeXYC(outputs, meta);
    }

    std::ostringstream oss;
    oss << "Unsupported RTMPose output layout:";
    for (std::size_t i = 0; i < outputs.size(); ++i) {
        oss << "\n  [" << i << "] name=\"" << outputs[i].name << "\" shape=[";
        const auto& shape = outputs[i].tensor.shape;
        for (std::size_t d = 0; d < shape.size(); ++d) {
            if (d != 0) {
                oss << ", ";
            }
            oss << shape[d];
        }
        oss << "]";
    }
    return Status::Error(StatusCode::Unsupported, oss.str());
}

Result<DecodedPoseWithDepth> DecodeRtmPoseOutputsWithDepth(
    const std::vector<NamedTensorF32>& outputs,
    const ImagePreprocessMeta& meta,
    const RtmPosePreprocessSpec& spec) {

    if (outputs.empty()) {
        return Status::Error(StatusCode::InvalidArgument,
            "DecodeRtmPoseOutputsWithDepth received zero outputs");
    }
    if (const auto s = ValidatePreprocessMeta(meta); !s.ok()) { return s; }
    if (const auto s = ValidatePreprocessSpec(spec);  !s.ok()) { return s; }

    if (LooksLikeRtmw3dWholeBodyOutput(outputs)) {
        return DecodeRtmw3dWholeBodySimCCFull(outputs, meta, spec);
    }

    // Non-3D model: decode 2D and return with pose3d.valid=false.
    DecodedPoseWithDepth result;
    result.pose3d.valid = false;
    result.pose3d.coordinate_frame = "model_simcc_body_relative";

    Result<DecodedPose2D> r2d{Status::Error(StatusCode::Unsupported, "unmatched")};
    if (LooksLikeRtmwWholeBodyOutput(outputs)) {
        r2d = DecodeRtmwWholeBodySimCC(outputs, meta, spec);
    } else if (LooksLikeSimCCOutput(outputs)) {
        r2d = DecodeSimCC(outputs, meta, spec);
    } else if (LooksLikeXYCOutput(outputs)) {
        r2d = DecodeXYC(outputs, meta);
    } else {
        std::ostringstream oss;
        oss << "Unsupported RTMPose output layout:";
        for (std::size_t i = 0; i < outputs.size(); ++i) {
            oss << "\n  [" << i << "] name=\"" << outputs[i].name << "\" shape=[";
            for (std::size_t d = 0; d < outputs[i].tensor.shape.size(); ++d) {
                if (d) oss << ", ";
                oss << outputs[i].tensor.shape[d];
            }
            oss << "]";
        }
        return Status::Error(StatusCode::Unsupported, oss.str());
    }
    if (!r2d.ok()) { return r2d.status(); }
    result.pose2d = r2d.value();
    return result;
}

std::string BuildPreprocessSummary(const ImagePreprocessMeta& meta) {
    std::ostringstream oss;
    oss << "source=" << meta.source_image_width << "x" << meta.source_image_height
        << " region=(" << meta.source_region.x << "," << meta.source_region.y << ","
        << meta.source_region.width << "," << meta.source_region.height << ")"
        << " model=" << meta.model_input_width << "x" << meta.model_input_height;
    return oss.str();
}

std::string BuildDecodedPoseSummary(const DecodedPose2D& pose) {
    std::ostringstream oss;
    oss << "valid=" << (pose.valid ? "true" : "false")
        << " format=" << ToString(pose.format)
        << " aggregate_confidence=" << pose.aggregate_confidence;
    return oss.str();
}

} // namespace bt
