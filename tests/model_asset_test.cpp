#include "test_check.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string ReadText(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool StrictModelAssetRequired() {
#if defined(_MSC_VER)
    char* value = nullptr;
    std::size_t value_size = 0;
    if (_dupenv_s(&value, &value_size, "BODYTRACKER_REQUIRE_ONNX_ASSET") != 0 || value == nullptr) {
        return false;
    }
    const std::string text(value);
    std::free(value);
    return text == "1";
#else
    const char* value = std::getenv("BODYTRACKER_REQUIRE_ONNX_ASSET");
    return value && std::string(value) == "1";
#endif
}

void TestModelMetadataReferencesConfiguredPath() {
    const std::filesystem::path model_path{"models/rtmw-dw-x-l-cocktail14-384x288.onnx"};
    const std::filesystem::path depth_model_path{"models/rtmw3d-x-cocktail14-384x288.onnx"};

    const auto config = ReadText("config/default.json");
    BT_CHECK(config.find(model_path.generic_string()) != std::string::npos);
    BT_CHECK(config.find(depth_model_path.generic_string()) != std::string::npos);
    BT_CHECK(config.find("\"depth_postprocess_enabled\": false") != std::string::npos);
    BT_CHECK(config.find("\"depth_postprocess_interval_frames\": 4") != std::string::npos);
    BT_CHECK(config.find("\"depth_postprocess_allow_cpu_fallback\": false") != std::string::npos);

    const std::filesystem::path readme_path{"models/README.md"};
    if (!std::filesystem::exists(readme_path)) {
        if (StrictModelAssetRequired()) {
            BT_CHECK(false);
        }
        std::cout << "Model README omitted from this source package; skipping packaged-model metadata check.\n";
        return;
    }

    const auto docs = ReadText(readme_path);
    BT_CHECK(docs.find(model_path.generic_string()) != std::string::npos);
    BT_CHECK(docs.find(depth_model_path.generic_string()) != std::string::npos);
    BT_CHECK(docs.find("input: [batch, 3, 384, 288]") != std::string::npos);
    BT_CHECK(docs.find("output x: [1, 133, 576]") != std::string::npos);
    BT_CHECK(docs.find("output y: [1, 133, 768]") != std::string::npos);
    BT_CHECK(docs.find("output z: [1, 133, 576]") != std::string::npos);
}

void TestModelBinaryWhenIncludedOrRequired() {
    const std::filesystem::path model_path{"models/rtmw-dw-x-l-cocktail14-384x288.onnx"};
    const std::filesystem::path depth_model_path{"models/rtmw3d-x-cocktail14-384x288.onnx"};
    const bool present = std::filesystem::exists(model_path);
    const bool depth_present = std::filesystem::exists(depth_model_path);

    if ((!present || !depth_present) && !StrictModelAssetRequired()) {
        std::cout << "ONNX model asset omitted from this source package; "
                     "metadata and hash sidecar remain checked. "
                     "Set BODYTRACKER_REQUIRE_ONNX_ASSET=1 for strict full-package validation.\n";
        return;
    }

    BT_CHECK(present);
    BT_CHECK(depth_present);
    BT_CHECK(std::filesystem::is_regular_file(model_path));
    BT_CHECK(std::filesystem::is_regular_file(depth_model_path));
    BT_CHECK(std::filesystem::file_size(model_path) > 100u * 1024u * 1024u);
    BT_CHECK(std::filesystem::file_size(depth_model_path) > 100u * 1024u * 1024u);
}

void TestProvidedModelHashSidecarIsPresent() {
    const std::filesystem::path sha_path{"models/rtmw-dw-x-l-cocktail14-384x288.onnx.sha256"};
    const std::filesystem::path depth_sha_path{"models/rtmw3d-x-cocktail14-384x288.onnx.sha256"};
    if (!std::filesystem::exists(sha_path)) {
        if (StrictModelAssetRequired()) {
            BT_CHECK(false);
        }
        std::cout << "Model hash sidecar omitted from this source package; skipping packaged-model hash check.\n";
        return;
    }
    const auto sha = ReadText(sha_path);
    BT_CHECK(sha.find("bd033156e5104c4f5d2edfe0453e02661e30a2f3da453ec93c8764d561b83054") != std::string::npos);
    BT_CHECK(sha.find("models/rtmw-dw-x-l-cocktail14-384x288.onnx") != std::string::npos);
    if (std::filesystem::exists(depth_sha_path)) {
        const auto depth_sha = ReadText(depth_sha_path);
        BT_CHECK(depth_sha.find("4a289c0e99d47eb595e99679d9d4a2d1def1b4241f9adcbafba44b9ff585ebcd") != std::string::npos);
        BT_CHECK(depth_sha.find("models/rtmw3d-x-cocktail14-384x288.onnx") != std::string::npos);
    } else if (StrictModelAssetRequired()) {
        BT_CHECK(false);
    }
}

} // namespace

int main() {
    TestModelMetadataReferencesConfiguredPath();
    TestProvidedModelHashSidecarIsPresent();
    TestModelBinaryWhenIncludedOrRequired();
    return 0;
}

