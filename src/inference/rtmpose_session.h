#pragma once

#include "core/status.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace bt {

enum class TensorElementType : std::uint8_t {
    Unknown = 0,
    Float32
};

const char* ToString(TensorElementType type);

struct TensorF32 {
    std::vector<std::int64_t> shape;
    std::vector<float> data;
};

struct NamedTensorF32 {
    std::string name;
    TensorF32 tensor;
};

struct ModelTensorInfo {
    std::string name;
    TensorElementType element_type = TensorElementType::Unknown;
    std::vector<std::int64_t> dims;
    bool is_tensor = false;
    bool has_dynamic_dims = false;
};

struct ModelSessionInfo {
    bool loaded = false;
    std::filesystem::path model_path;
    std::string active_device = "directml";
    bool ep_fallback = false;
    bool cpu_fallback_allowed = true;
    std::vector<ModelTensorInfo> inputs;
    std::vector<ModelTensorInfo> outputs;
};

class RtmPoseSession {
public:
    RtmPoseSession();
    ~RtmPoseSession();

    RtmPoseSession(const RtmPoseSession&) = delete;
    RtmPoseSession& operator=(const RtmPoseSession&) = delete;
    RtmPoseSession(RtmPoseSession&&) noexcept;
    RtmPoseSession& operator=(RtmPoseSession&&) noexcept;

    Status Load(const std::filesystem::path& model_path, const std::string& device = "directml");
    [[nodiscard]] const ModelSessionInfo& GetInfo() const noexcept;
    [[nodiscard]] bool HasStaticSingleInputShape() const;
    Result<TensorF32> MakeZeroInputTensorForStaticShape() const;
    Result<std::vector<NamedTensorF32>> RunSingleInputF32(const TensorF32& input);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    ModelSessionInfo info_{};
};

std::string BuildModelSessionSummary(const ModelSessionInfo& info);

} // namespace bt
