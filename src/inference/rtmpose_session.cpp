#include "inference/rtmpose_session.h"

#include "core/logging.h"

#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <cstring>
#include <numeric>
#include <sstream>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#endif

namespace bt {
namespace {

TensorElementType ToTensorElementType(ONNXTensorElementDataType type) {
    switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
        return TensorElementType::Float32;
    default:
        return TensorElementType::Unknown;
    }
}

std::size_t ElementCountFromShape(const std::vector<std::int64_t>& shape) {
    std::size_t count = 1;
    for (const auto dim : shape) {
        if (dim <= 0) {
            return 0;
        }
        count *= static_cast<std::size_t>(dim);
    }
    return count;
}

bool RuntimeShapeCompatibleWithModelShape(
    const std::vector<std::int64_t>& runtime_shape,
    const std::vector<std::int64_t>& model_shape) {
    if (runtime_shape.size() != model_shape.size()) {
        return false;
    }
    for (std::size_t i = 0; i < runtime_shape.size(); ++i) {
        if (runtime_shape[i] <= 0) {
            return false;
        }
        if (model_shape[i] > 0 && runtime_shape[i] != model_shape[i]) {
            return false;
        }
    }
    return true;
}

bool HasConcreteSingleImageInputShape(const ModelSessionInfo& info) {
    if (!info.loaded || info.inputs.size() != 1 || !info.inputs.front().is_tensor) {
        return false;
    }
    const auto& dims = info.inputs.front().dims;
    return dims.size() == 4 &&
        (dims[0] == 1 || dims[0] <= 0) &&
        dims[1] == 3 &&
        dims[2] > 0 &&
        dims[3] > 0;
}

std::string ShapeToString(const std::vector<std::int64_t>& shape) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < shape.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << shape[i];
    }
    oss << "]";
    return oss.str();
}

Status AppendDirectMlExecutionProvider(Ort::SessionOptions& session_options) {
#ifdef _WIN32
    using AppendDmlFn = OrtStatus*(ORT_API_CALL*)(OrtSessionOptions*, int);

    HMODULE module = GetModuleHandleW(L"onnxruntime.dll");
    if (module == nullptr) {
        module = LoadLibraryW(L"onnxruntime.dll");
    }
    if (module == nullptr) {
        return Status::Error(StatusCode::InternalError, "onnxruntime.dll is not loaded");
    }

    const auto append_dml = reinterpret_cast<AppendDmlFn>(
        GetProcAddress(module, "OrtSessionOptionsAppendExecutionProvider_DML"));
    if (append_dml == nullptr) {
        return Status::Error(
            StatusCode::InternalError,
            "ONNX Runtime DirectML execution provider entry point is unavailable");
    }

    OrtStatus* status = append_dml(session_options, 0);
    if (status != nullptr) {
        const OrtApi& api = Ort::GetApi();
        std::string message = api.GetErrorMessage(status);
        api.ReleaseStatus(status);
        return Status::Error(StatusCode::InternalError, message);
    }
    return Status::OK();
#else
    (void)session_options;
    return Status::Error(StatusCode::InternalError, "DirectML execution provider is only available on Windows");
#endif
}

} // namespace

class RtmPoseSession::Impl {
public:
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "bodytracker-rtmpose"};
    Ort::SessionOptions session_options{};
    Ort::MemoryInfo memory_info{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    Ort::RunOptions run_options{nullptr};
    std::unique_ptr<Ort::Session> session;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;

    void RebuildNamePointers() {
        input_name_ptrs.clear();
        input_name_ptrs.reserve(input_names.size());
        for (const auto& name : input_names) {
            input_name_ptrs.push_back(name.c_str());
        }
        output_name_ptrs.clear();
        output_name_ptrs.reserve(output_names.size());
        for (const auto& name : output_names) {
            output_name_ptrs.push_back(name.c_str());
        }
    }
};

const char* ToString(TensorElementType type) {
    switch (type) {
    case TensorElementType::Float32: return "float32";
    default: return "unknown";
    }
}

RtmPoseSession::RtmPoseSession()
    : impl_(std::make_unique<Impl>()) {
}

RtmPoseSession::~RtmPoseSession() = default;
RtmPoseSession::RtmPoseSession(RtmPoseSession&&) noexcept = default;
RtmPoseSession& RtmPoseSession::operator=(RtmPoseSession&&) noexcept = default;

Status RtmPoseSession::Load(const std::filesystem::path& model_path, const std::string& device) {
    if (!std::filesystem::exists(model_path)) {
        return Status::Error(StatusCode::InvalidArgument, "Model file not found: " + model_path.string());
    }

    try {
        impl_ = std::make_unique<Impl>();
        const bool request_directml = device == "directml" || device == "directml_strict";
        const bool allow_cpu_fallback = device != "directml_strict";
        impl_->session_options.SetGraphOptimizationLevel(
            request_directml ? GraphOptimizationLevel::ORT_DISABLE_ALL : GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        std::string active_device = "cpu";
        bool ep_fallback = false;
        if (request_directml) {
            if (const auto dml = AppendDirectMlExecutionProvider(impl_->session_options); dml.ok()) {
                active_device = "directml";
            } else {
                if (!allow_cpu_fallback) {
                    return dml;
                }
                ep_fallback = true;
                // The session will run on CPU; restore the CPU optimization level
                // instead of keeping the DirectML-specific ORT_DISABLE_ALL, which
                // would silently run the CPU path unoptimized.
                impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
                bt::Logger::Instance().Write(
                    bt::LogLevel::Warn,
                    "DirectML execution provider unavailable; falling back to CPU: " + dml.message);
            }
        }

        try {
            impl_->session = std::make_unique<Ort::Session>(impl_->env, model_path.c_str(), impl_->session_options);
        } catch (const Ort::Exception& e) {
            if (active_device != "directml" || !allow_cpu_fallback) {
                throw;
            }
            ep_fallback = true;
            active_device = "cpu";
            bt::Logger::Instance().Write(
                bt::LogLevel::Warn,
                std::string("DirectML session creation failed; falling back to CPU: ") + e.what());
            impl_ = std::make_unique<Impl>();
            impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
            impl_->session = std::make_unique<Ort::Session>(impl_->env, model_path.c_str(), impl_->session_options);
        }

        Ort::AllocatorWithDefaultOptions allocator;
        ModelSessionInfo next_info;
        next_info.loaded = true;
        next_info.model_path = model_path;
        next_info.active_device = active_device;
        next_info.ep_fallback = ep_fallback;
        next_info.cpu_fallback_allowed = allow_cpu_fallback;

        const auto read_tensor_info = [](Ort::TypeInfo& type_info) {
            ModelTensorInfo tensor_info;
            tensor_info.is_tensor = type_info.GetONNXType() == ONNX_TYPE_TENSOR;
            if (!tensor_info.is_tensor) {
                return tensor_info;
            }

            const auto shape_info = type_info.GetTensorTypeAndShapeInfo();
            tensor_info.element_type = ToTensorElementType(shape_info.GetElementType());
            tensor_info.dims = shape_info.GetShape();
            tensor_info.has_dynamic_dims = std::any_of(
                tensor_info.dims.begin(),
                tensor_info.dims.end(),
                [](std::int64_t dim) { return dim <= 0; });
            return tensor_info;
        };

        impl_->input_names.clear();
        const std::size_t input_count = impl_->session->GetInputCount();
        for (std::size_t i = 0; i < input_count; ++i) {
            auto name = impl_->session->GetInputNameAllocated(i, allocator);
            auto type_info = impl_->session->GetInputTypeInfo(i);
            auto tensor_info = read_tensor_info(type_info);
            tensor_info.name = name.get() ? name.get() : "";
            impl_->input_names.push_back(tensor_info.name);
            next_info.inputs.push_back(std::move(tensor_info));
        }

        impl_->output_names.clear();
        const std::size_t output_count = impl_->session->GetOutputCount();
        for (std::size_t i = 0; i < output_count; ++i) {
            auto name = impl_->session->GetOutputNameAllocated(i, allocator);
            auto type_info = impl_->session->GetOutputTypeInfo(i);
            auto tensor_info = read_tensor_info(type_info);
            tensor_info.name = name.get() ? name.get() : "";
            impl_->output_names.push_back(tensor_info.name);
            next_info.outputs.push_back(std::move(tensor_info));
        }
        impl_->RebuildNamePointers();

        info_ = std::move(next_info);
        if (const auto zero = MakeZeroInputTensorForStaticShape(); zero.ok()) {
            const auto warmup = RunSingleInputF32(zero.value());
            if (!warmup.ok()) {
                const auto status = warmup.status();
                info_ = {};
                impl_->session.reset();
                return status;
            }
            if (warmup.value().size() == info_.outputs.size()) {
                for (std::size_t i = 0; i < info_.outputs.size(); ++i) {
                    info_.outputs[i].dims = warmup.value()[i].tensor.shape;
                    info_.outputs[i].has_dynamic_dims = std::any_of(
                        info_.outputs[i].dims.begin(),
                        info_.outputs[i].dims.end(),
                        [](std::int64_t dim) { return dim <= 0; });
                }
            }
        }
        return Status::OK();
    } catch (const Ort::Exception& e) {
        info_ = {};
        impl_->session.reset();
        return Status::Error(StatusCode::InternalError, std::string("ONNX Runtime model load failed: ") + e.what());
    } catch (const std::exception& e) {
        info_ = {};
        impl_->session.reset();
        return Status::Error(StatusCode::InternalError, std::string("Model load failed: ") + e.what());
    }
}

const ModelSessionInfo& RtmPoseSession::GetInfo() const noexcept {
    return info_;
}

bool RtmPoseSession::HasStaticSingleInputShape() const {
    return HasConcreteSingleImageInputShape(info_);
}

Result<TensorF32> RtmPoseSession::MakeZeroInputTensorForStaticShape() const {
    if (!HasStaticSingleInputShape()) {
        return Status::Error(StatusCode::FailedPrecondition, "Model does not have a concrete single-image input shape");
    }

    TensorF32 t;
    t.shape = info_.inputs.front().dims;
    if (t.shape[0] <= 0) {
        t.shape[0] = 1;
    }
    std::int64_t count = 1;
    for (const auto dim : t.shape) {
        if (dim <= 0) {
            return Status::Error(StatusCode::ValidationError, "Static input shape contains non-positive dimension");
        }
        count *= dim;
    }
    t.data.assign(static_cast<std::size_t>(count), 0.0f);
    return t;
}

Result<std::vector<NamedTensorF32>> RtmPoseSession::RunSingleInputF32(const TensorF32& input) {
    if (!info_.loaded) {
        return Status::Error(StatusCode::FailedPrecondition, "Model session is not loaded");
    }
    if (!impl_ || !impl_->session) {
        return Status::Error(StatusCode::FailedPrecondition, "ONNX Runtime session is not initialized");
    }
    if (input.data.empty()) {
        return Status::Error(StatusCode::InvalidArgument, "Input tensor is empty");
    }
    if (info_.inputs.size() != 1) {
        return Status::Error(StatusCode::ValidationError, "RunSingleInputF32 requires exactly one model input");
    }
    if (!RuntimeShapeCompatibleWithModelShape(input.shape, info_.inputs.front().dims)) {
        return Status::Error(
            StatusCode::ValidationError,
            "Input tensor shape " + ShapeToString(input.shape) +
                " is not compatible with model input shape " + ShapeToString(info_.inputs.front().dims));
    }
    const auto expected_count = ElementCountFromShape(input.shape);
    if (expected_count == 0 || input.data.size() != expected_count) {
        return Status::Error(StatusCode::ValidationError, "Input tensor data size does not match input shape");
    }

    try {
        auto input_tensor = Ort::Value::CreateTensor<float>(
            impl_->memory_info,
            const_cast<float*>(input.data.data()),
            input.data.size(),
            input.shape.data(),
            input.shape.size());

        auto ort_outputs = impl_->session->Run(
            impl_->run_options,
            impl_->input_name_ptrs.data(),
            &input_tensor,
            1,
            impl_->output_name_ptrs.data(),
            impl_->output_name_ptrs.size());

        std::vector<NamedTensorF32> outputs;
        outputs.reserve(ort_outputs.size());
        for (std::size_t i = 0; i < ort_outputs.size(); ++i) {
            if (!ort_outputs[i].IsTensor()) {
                return Status::Error(StatusCode::ValidationError, "ONNX output is not a tensor: " + impl_->output_names[i]);
            }

            const auto type_info = ort_outputs[i].GetTensorTypeAndShapeInfo();
            if (type_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                return Status::Error(StatusCode::ValidationError, "ONNX output is not float32: " + impl_->output_names[i]);
            }

            NamedTensorF32 out;
            out.name = impl_->output_names[i];
            out.tensor.shape = type_info.GetShape();
            const auto count = type_info.GetElementCount();
            const float* data = ort_outputs[i].GetTensorData<float>();
            out.tensor.data.assign(data, data + count);
            outputs.push_back(std::move(out));
        }

        return outputs;
    } catch (const Ort::Exception& e) {
        if (info_.active_device == "directml" && info_.cpu_fallback_allowed) {
            const auto original_path = info_.model_path;
            const std::string original_error = e.what();
            bt::Logger::Instance().Write(
                bt::LogLevel::Warn,
                "DirectML inference failed; falling back to CPU for this model: " + original_error);
            if (const auto reload = Load(original_path, "cpu"); !reload.ok()) {
                return Status::Error(
                    StatusCode::InternalError,
                    "ONNX Runtime inference failed on DirectML and CPU fallback load failed: " +
                        original_error + "; CPU fallback: " + reload.message);
            }
            info_.ep_fallback = true;
            return RunSingleInputF32(input);
        }
        return Status::Error(StatusCode::InternalError, std::string("ONNX Runtime inference failed: ") + e.what());
    }
}

std::string BuildModelSessionSummary(const ModelSessionInfo& info) {
    std::ostringstream oss;
    oss << "model_loaded=" << (info.loaded ? "true" : "false")
        << " path=" << info.model_path.string()
        << " active_device=" << info.active_device
        << " ep_fallback=" << (info.ep_fallback ? "true" : "false")
        << " inputs=" << info.inputs.size()
        << " outputs=" << info.outputs.size();
    return oss.str();
}

} // namespace bt
