#pragma once

#include "core/status.h"
#include "debug/debug_snapshot.h"

#include <filesystem>
#include <string>
#include <vector>

namespace bt {

struct ReplayRecordingMetadata {
    int schema_version = 1;
    std::string format;
    bool deterministic = false;
    std::string config_path;
    std::string config_json;
    std::string config_hash;
    std::string model_path;
    std::string calibration_path;
    std::size_t frame_count = 0;
    double first_timestamp_seconds = 0.0;
    double last_timestamp_seconds = 0.0;
};

struct ReplayRecording {
    ReplayRecordingMetadata metadata{};
    std::vector<DebugSnapshot> frames{};
};

class ReplayPlayer {
public:
    Result<std::vector<DebugSnapshot>> Load(const std::filesystem::path& path);
    Result<ReplayRecording> LoadRecording(const std::filesystem::path& path);
    Result<DebugSnapshot> SeekAtOrAfter(const ReplayRecording& recording, double timestamp_seconds) const;
};

} // namespace bt
