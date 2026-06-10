#pragma once

#include "core/status.h"
#include "debug/debug_snapshot.h"

#include <filesystem>
#include <fstream>
#include <cstdint>
#include <string>

namespace bt {

struct ReplayLogSessionInfo {
    int schema_version = 3;
    std::string config_path;
    std::string config_json;
    std::string model_path;
    std::string calibration_path;
    bool deterministic = true;
};

class ReplayLogWriter {
public:
    Status Open(const std::filesystem::path& path);
    Status Open(const std::filesystem::path& path, const ReplayLogSessionInfo& session);
    Status WriteSnapshot(const DebugSnapshot& snapshot);
    Status Append(const DebugSnapshot& snapshot);
    void Close();

private:
    std::filesystem::path path_;
    ReplayLogSessionInfo session_{};
    std::uint64_t frame_index_ = 0;
    std::ofstream out_{};
    bool open_ = false;
};

} // namespace bt
