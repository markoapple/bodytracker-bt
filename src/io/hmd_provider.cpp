#include "io/hmd_provider.h"

#include <cmath>
#include <exception>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <utility>

namespace bt {
namespace {

Result<Vec3f> ReadRequiredVec3(const nlohmann::json& j, const char* key) {
    if (!j.is_object() || !j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3) {
        std::ostringstream oss;
        oss << "HMD pose JSON key \"" << key << "\" must be an array of 3 numbers";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    const auto& a = j.at(key);
    if (!a[0].is_number() || !a[1].is_number() || !a[2].is_number()) {
        std::ostringstream oss;
        oss << "HMD pose JSON key \"" << key << "\" must contain only numbers";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    Vec3f out{a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z)) {
        std::ostringstream oss;
        oss << "HMD pose JSON key \"" << key << "\" contains a non-finite value";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    return out;
}

Result<Quatf> ReadRequiredQuat(const nlohmann::json& j, const char* key) {
    if (!j.is_object() || !j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 4) {
        std::ostringstream oss;
        oss << "HMD pose JSON key \"" << key << "\" must be an array of 4 numbers";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    const auto& a = j.at(key);
    if (!a[0].is_number() || !a[1].is_number() || !a[2].is_number() || !a[3].is_number()) {
        std::ostringstream oss;
        oss << "HMD pose JSON key \"" << key << "\" must contain only numbers";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    Quatf out{a[0].get<float>(), a[1].get<float>(), a[2].get<float>(), a[3].get<float>()};
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z) || !std::isfinite(out.w)) {
        std::ostringstream oss;
        oss << "HMD pose JSON key \"" << key << "\" contains a non-finite value";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }
    const float len = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z + out.w * out.w);
    if (!std::isfinite(len) || len < 1e-6f) {
        return Status::Error(StatusCode::ValidationError, "HMD pose orientation quaternion must be non-zero");
    }
    return Normalize(out);
}

Result<double> ReadTimestampSecondsOr(const nlohmann::json& j, double fallback) {
    if (!j.is_object() || !j.contains("timestamp_seconds")) {
        return fallback;
    }
    if (!j.at("timestamp_seconds").is_number()) {
        return Status::Error(StatusCode::ValidationError, "HMD pose timestamp_seconds must be a number");
    }
    const double timestamp = j.at("timestamp_seconds").get<double>();
    if (!std::isfinite(timestamp)) {
        return Status::Error(StatusCode::ValidationError, "HMD pose timestamp_seconds must be finite");
    }
    return timestamp;
}

Result<bool> ReadValidOr(const nlohmann::json& j, bool fallback) {
    if (!j.is_object() || !j.contains("valid")) {
        return fallback;
    }
    if (!j.at("valid").is_boolean()) {
        return Status::Error(StatusCode::ValidationError, "HMD pose valid must be a boolean");
    }
    return j.at("valid").get<bool>();
}

} // namespace

Result<HmdPoseSample> NullHmdProvider::Poll(double target_time_seconds) {
    HmdPoseSample sample;
    sample.timestamp_seconds = target_time_seconds;
    sample.valid = false;
    return sample;
}

JsonFileHmdProvider::JsonFileHmdProvider(HmdProviderConfig config)
    : config_(std::move(config)) {
}

Result<HmdPoseSample> JsonFileHmdProvider::Poll(double target_time_seconds) {
    std::ifstream in(config_.pose_json_path);
    if (!in) {
        HmdPoseSample sample;
        sample.timestamp_seconds = target_time_seconds;
        sample.valid = false;
        return sample;
    }

    nlohmann::json j;
    try {
        in >> j;

        const auto position = ReadRequiredVec3(j, "position");
        if (!position.ok()) {
            return position.status();
        }
        const auto orientation = ReadRequiredQuat(j, "orientation");
        if (!orientation.ok()) {
            return orientation.status();
        }
        const auto timestamp = ReadTimestampSecondsOr(j, target_time_seconds);
        if (!timestamp.ok()) {
            return timestamp.status();
        }
        const auto valid = ReadValidOr(j, true);
        if (!valid.ok()) {
            return valid.status();
        }

        HmdPoseSample sample;
        sample.pose.position = position.value();
        sample.pose.orientation = orientation.value();
        sample.timestamp_seconds = timestamp.value();
        sample.valid = valid.value();
        return sample;
    } catch (const std::exception& e) {
        return Status::Error(StatusCode::ValidationError, std::string("HMD pose JSON load failed: ") + e.what());
    }
}


std::unique_ptr<IHmdProvider> MakeHmdProvider(const HmdProviderConfig& config) {
    if (config.mode == "json_file") {
        return std::make_unique<JsonFileHmdProvider>(config);
    }
    if (config.mode == "steamvr") {
        // SteamVR is still used for controller tracker-space alignment and virtual tracker output.
        // It is intentionally not an HMD body-depth provider; camera/depth paths own 3D body solving.
        return std::make_unique<NullHmdProvider>();
    }
    return std::make_unique<NullHmdProvider>();
}

} // namespace bt
