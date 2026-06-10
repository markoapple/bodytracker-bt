#include "io/hmd_provider.h"
#include "test_check.h"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

namespace {

std::filesystem::path TempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / name;
}

void WriteFile(const std::filesystem::path& path, const std::string& content) {
    std::ofstream out(path, std::ios::trunc);
    BT_CHECK(out.good());
    out << content;
}

std::unique_ptr<bt::IHmdProvider> JsonProvider(const std::filesystem::path& path) {
    bt::HmdProviderConfig cfg;
    cfg.mode = "json_file";
    cfg.pose_json_path = path;
    return bt::MakeHmdProvider(cfg);
}

} // namespace

int main() {
    const auto valid_path = TempPath("bodytracker_hmd_valid.json");
    WriteFile(valid_path, R"JSON({
  "position": [1.0, 2.0, 3.0],
  "orientation": [0.0, 0.0, 0.0, 2.0],
  "timestamp_seconds": 42.5,
  "valid": true
})JSON");

    auto valid_provider = JsonProvider(valid_path);
    const auto valid = valid_provider->Poll(1.0);
    BT_CHECK(valid.ok());
    BT_CHECK(valid.value().valid);
    BT_CHECK_NEAR(valid.value().pose.position.x, 1.0, 1e-6);
    BT_CHECK_NEAR(valid.value().pose.position.y, 2.0, 1e-6);
    BT_CHECK_NEAR(valid.value().pose.position.z, 3.0, 1e-6);
    BT_CHECK_NEAR(valid.value().pose.orientation.w, 1.0, 1e-6);
    BT_CHECK_NEAR(valid.value().timestamp_seconds, 42.5, 1e-6);

    bt::HmdProviderConfig steamvr_cfg;
    steamvr_cfg.mode = "steamvr";
    auto steamvr_provider = bt::MakeHmdProvider(steamvr_cfg);
    const auto steamvr_sample = steamvr_provider->Poll(123.0);
    BT_CHECK(steamvr_sample.ok());
    BT_CHECK(!steamvr_sample.value().valid);
    BT_CHECK_NEAR(steamvr_sample.value().timestamp_seconds, 123.0, 1e-6);

    const auto missing_path = TempPath("bodytracker_hmd_missing.json");
    std::filesystem::remove(missing_path);
    auto missing_provider = JsonProvider(missing_path);
    const auto missing = missing_provider->Poll(12.0);
    BT_CHECK(missing.ok());
    BT_CHECK(!missing.value().valid);
    BT_CHECK_NEAR(missing.value().timestamp_seconds, 12.0, 1e-6);

    const auto malformed_path = TempPath("bodytracker_hmd_malformed.json");
    WriteFile(malformed_path, R"JSON({
  "position": [1.0, 2.0],
  "orientation": [0.0, 0.0, 0.0, 1.0]
})JSON");
    auto malformed_provider = JsonProvider(malformed_path);
    const auto malformed = malformed_provider->Poll(1.0);
    BT_CHECK(!malformed.ok());

    const auto zero_quat_path = TempPath("bodytracker_hmd_zero_quat.json");
    WriteFile(zero_quat_path, R"JSON({
  "position": [1.0, 2.0, 3.0],
  "orientation": [0.0, 0.0, 0.0, 0.0]
})JSON");
    auto zero_quat_provider = JsonProvider(zero_quat_path);
    const auto zero_quat = zero_quat_provider->Poll(1.0);
    BT_CHECK(!zero_quat.ok());

    const auto invalid_flag_path = TempPath("bodytracker_hmd_invalid_flag.json");
    WriteFile(invalid_flag_path, R"JSON({
  "position": [1.0, 2.0, 3.0],
  "orientation": [0.0, 0.0, 0.0, 1.0],
  "valid": false
})JSON");
    auto invalid_flag_provider = JsonProvider(invalid_flag_path);
    const auto invalid_flag = invalid_flag_provider->Poll(99.0);
    BT_CHECK(invalid_flag.ok());
    BT_CHECK(!invalid_flag.value().valid);
    BT_CHECK_NEAR(invalid_flag.value().timestamp_seconds, 99.0, 1e-6);

    std::filesystem::remove(valid_path);
    std::filesystem::remove(malformed_path);
    std::filesystem::remove(zero_quat_path);
    std::filesystem::remove(invalid_flag_path);
    return 0;
}
