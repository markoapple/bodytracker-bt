#include "inference/rtmpose_model_contract.h"

#include "core/types.h"
#include "inference/keypoint_contract.h"

#include <sstream>

namespace bt {
namespace {

std::string ShapeToString(const std::vector<std::int64_t>& dims) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < dims.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << dims[i];
    }
    oss << "]";
    return oss.str();
}

Status RequireFloatTensor(const ModelTensorInfo& tensor, const char* role) {
    if (!tensor.is_tensor) {
        return Status::Error(StatusCode::ValidationError, std::string("RTMPose ") + role + " must be a tensor: " + tensor.name);
    }
    if (tensor.element_type != TensorElementType::Float32) {
        return Status::Error(StatusCode::ValidationError, std::string("RTMPose ") + role + " must be float32: " + tensor.name);
    }
    return Status::OK();
}

bool IsConcreteOrDynamicBatch(std::int64_t dim) {
    return dim == 1 || dim <= 0;
}

bool StaticDimEquals(const ModelTensorInfo& tensor, std::size_t index, std::int64_t expected) {
    return index < tensor.dims.size() && tensor.dims[index] == expected;
}

Status RequireOutputName(const ModelTensorInfo& tensor, const char* expected_name) {
    if (tensor.name != expected_name) {
        return Status::Error(
            StatusCode::ValidationError,
            std::string("RTMPose SimCC output name mismatch: expected ") +
                expected_name + ", got " + tensor.name);
    }
    return Status::OK();
}

Status ValidateSimCCOutputPair(const ModelSessionInfo& info) {
    const auto& x = info.outputs[0];
    const auto& y = info.outputs[1];
    if (const auto s = RequireFloatTensor(x, "SimCC output"); !s.ok()) {
        return s;
    }
    if (const auto s = RequireFloatTensor(y, "SimCC output"); !s.ok()) {
        return s;
    }
    if (x.dims.size() != 3 || y.dims.size() != 3) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMPose SimCC output must be two rank-3 tensors [1, keypoints, bins]; got " +
                ShapeToString(x.dims) + " and " + ShapeToString(y.dims));
    }
    if (!IsConcreteOrDynamicBatch(x.dims[0]) || !IsConcreteOrDynamicBatch(y.dims[0])) {
        return Status::Error(StatusCode::ValidationError, "RTMPose SimCC batch dimension must be dynamic or 1");
    }

    const bool is_cocktail14 =
        StaticDimEquals(x, 1, static_cast<std::int64_t>(kHalpe26Count)) &&
        StaticDimEquals(y, 1, static_cast<std::int64_t>(kHalpe26Count));
    const bool is_wholebody133 =
        StaticDimEquals(x, 1, kRtmw3dWholeBodyKeypointCount) &&
        StaticDimEquals(y, 1, kRtmw3dWholeBodyKeypointCount);
    if (!is_cocktail14 && !is_wholebody133) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMPose SimCC output must expose 26 keypoints (Cocktail14 mapped) or 133 COCO-WholeBody keypoints after runtime shape observation; got " +
                ShapeToString(x.dims) + " and " + ShapeToString(y.dims));
    }
    if (is_cocktail14) {
        if (const auto s = RequireOutputName(x, "simcc_x"); !s.ok()) {
            return s;
        }
        if (const auto s = RequireOutputName(y, "simcc_y"); !s.ok()) {
            return s;
        }
    }
    if (!StaticDimEquals(x, 2, kRtmPoseHalpe26SimccXBins) ||
        !StaticDimEquals(y, 2, kRtmPoseHalpe26SimccYBins)) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMPose SimCC bins must be [x=576, y=768] for input [1, 3, 384, 288]; got " +
                ShapeToString(x.dims) + " and " + ShapeToString(y.dims));
    }
    return Status::OK();
}

Status ValidateXYCOutput(const ModelSessionInfo& info) {
    const auto& out = info.outputs.front();
    if (const auto s = RequireFloatTensor(out, "XYC output"); !s.ok()) {
        return s;
    }
    if (out.dims.size() == 3) {
        if (!IsConcreteOrDynamicBatch(out.dims[0]) ||
            !StaticDimEquals(out, 1, static_cast<std::int64_t>(kHalpe26Count)) ||
            out.dims[2] < 3) {
            return Status::Error(StatusCode::ValidationError,
                "RTMPose-X XYC output must be [1, 26, >=3]; got " + ShapeToString(out.dims));
        }
        return Status::OK();
    }
    if (out.dims.size() == 2) {
        if (!StaticDimEquals(out, 0, static_cast<std::int64_t>(kHalpe26Count)) || out.dims[1] < 3) {
            return Status::Error(StatusCode::ValidationError,
                "RTMPose-X XYC output must be [26, >=3]; got " + ShapeToString(out.dims));
        }
        return Status::OK();
    }
    return Status::Error(StatusCode::ValidationError,
        "RTMPose-X XYC output must be rank 2 or 3; got " + ShapeToString(out.dims));
}

Status ValidateRtmw3dWholeBodyOutput(const ModelSessionInfo& info) {
    const auto& x = info.outputs[0];
    const auto& y = info.outputs[1];
    const auto& z = info.outputs[2];
    if (const auto s = RequireFloatTensor(x, "RTMW3D x output"); !s.ok()) {
        return s;
    }
    if (const auto s = RequireFloatTensor(y, "RTMW3D y output"); !s.ok()) {
        return s;
    }
    if (const auto s = RequireFloatTensor(z, "RTMW3D z output"); !s.ok()) {
        return s;
    }
    if (x.dims.size() != 3 || y.dims.size() != 3 || z.dims.size() != 3) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMW3D whole-body output must be three rank-3 tensors [1, 133, x/y/z bins]; got " +
                ShapeToString(x.dims) + ", " + ShapeToString(y.dims) + ", " + ShapeToString(z.dims));
    }
    if (!IsConcreteOrDynamicBatch(x.dims[0]) ||
        !IsConcreteOrDynamicBatch(y.dims[0]) ||
        !IsConcreteOrDynamicBatch(z.dims[0])) {
        return Status::Error(StatusCode::ValidationError, "RTMW3D whole-body batch dimension must be dynamic or 1");
    }
    if (!StaticDimEquals(x, 1, kRtmw3dWholeBodyKeypointCount) ||
        !StaticDimEquals(y, 1, kRtmw3dWholeBodyKeypointCount) ||
        !StaticDimEquals(z, 1, kRtmw3dWholeBodyKeypointCount)) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMW3D whole-body output must expose 133 COCO-WholeBody keypoints; got " +
                ShapeToString(x.dims) + ", " + ShapeToString(y.dims) + ", " + ShapeToString(z.dims));
    }
    if (!StaticDimEquals(x, 2, kRtmPoseHalpe26SimccXBins) ||
        !StaticDimEquals(y, 2, kRtmPoseHalpe26SimccYBins) ||
        !StaticDimEquals(z, 2, kRtmw3dSimccZBins)) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMW3D whole-body SimCC bins must be [x=576, y=768, z=576] for input [1, 3, 384, 288]; got " +
                ShapeToString(x.dims) + ", " + ShapeToString(y.dims) + ", " + ShapeToString(z.dims));
    }
    return Status::OK();
}

} // namespace

Status ValidateRtmPoseImageInputContract(const ModelSessionInfo& info) {
    if (!info.loaded) {
        return Status::Error(StatusCode::FailedPrecondition, "Model session is not loaded");
    }
    if (!InternalKeypointOrderIsCanonical()) {
        return Status::Error(StatusCode::InternalError, "Internal KeypointId enum no longer matches canonical internal keypoint order");
    }
    if (info.inputs.size() != 1) {
        return Status::Error(StatusCode::ValidationError, "RTMPose expects exactly one image input tensor");
    }
    const auto& input = info.inputs.front();
    if (const auto s = RequireFloatTensor(input, "image input"); !s.ok()) {
        return s;
    }
    if (input.name != "input") {
        return Status::Error(StatusCode::ValidationError, "RTMPose input name must be input; got " + input.name);
    }
    if (input.dims.size() != 4) {
        return Status::Error(StatusCode::ValidationError, "RTMPose image input must be a rank-4 NCHW tensor");
    }
    if (!IsConcreteOrDynamicBatch(input.dims[0]) || input.dims[1] != 3) {
        return Status::Error(
            StatusCode::Unsupported,
            "RTMPose input path requires float32 NCHW input [batch, 3, 384, 288]");
    }
    if (input.dims[2] != kRtmPoseHalpe26InputHeight || input.dims[3] != kRtmPoseHalpe26InputWidth) {
        return Status::Error(
            StatusCode::ValidationError,
            "RTMPose input must be [batch, 3, 384, 288] NCHW; got " + ShapeToString(input.dims));
    }
    return Status::OK();
}

Status ValidateRtmPoseOutputContract(const ModelSessionInfo& info) {
    if (!info.loaded) {
        return Status::Error(StatusCode::FailedPrecondition, "Model session is not loaded");
    }
    if (info.outputs.size() == 2) {
        return ValidateSimCCOutputPair(info);
    }
    if (info.outputs.size() == 1) {
        return ValidateXYCOutput(info);
    }
    if (info.outputs.size() == 3) {
        return ValidateRtmw3dWholeBodyOutput(info);
    }
    return Status::Error(
        StatusCode::ValidationError,
        "Supported pose exports must expose RTMPose Cocktail14 SimCC/XYC, RTMW whole-body SimCC, or RTMW3D whole-body SimCC; got " +
            std::to_string(info.outputs.size()) + " outputs");
}

Status ValidateRtmPoseModelContract(const ModelSessionInfo& info) {
    if (const auto s = ValidateRtmPoseImageInputContract(info); !s.ok()) {
        return s;
    }
    return ValidateRtmPoseOutputContract(info);
}

} // namespace bt
