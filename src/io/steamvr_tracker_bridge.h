#pragma once

#include "core/config.h"
#include "core/status.h"
#include "tracking/tracker_synthesis.h"

#include <array>
#include <cstdint>
#include <string>

namespace bt {

struct SteamVrTrackerBridgeReport {
    bool enabled = false;
    bool open = false;
    bool last_send_ok = true;
    std::string status = "disabled";
    std::string last_error;
    int sent_tracker_count = 0;
    int skipped_tracker_count = 0;
    int sent_message_count = 0;
    std::uint64_t sequence = 0;
    std::string target_address;
    int target_port = 0;
    float min_confidence = 0.0f;
    std::array<bool, kTrackerPoseCount> role_enabled{};
    std::array<bool, kTrackerPoseCount> role_valid{};
    std::array<bool, kTrackerPoseCount> role_sent{};
    std::array<bool, kTrackerPoseCount> role_degraded{};
    std::array<float, kTrackerPoseCount> role_confidence{};
    std::array<std::string, kTrackerPoseCount> role_reasons{};
};

class SteamVrTrackerBridgeSender {
public:
    explicit SteamVrTrackerBridgeSender(SteamVrTrackerBridgeConfig config);
    ~SteamVrTrackerBridgeSender();

    SteamVrTrackerBridgeSender(const SteamVrTrackerBridgeSender&) = delete;
    SteamVrTrackerBridgeSender& operator=(const SteamVrTrackerBridgeSender&) = delete;

    Status Open();
    void Close();
    Status UpdateConfig(const SteamVrTrackerBridgeConfig& next_config);
    Status SendTrackers(const TrackerPoseArray& trackers, const OscConfig& tracker_space, double timestamp_seconds);

    [[nodiscard]] bool IsOpen() const noexcept { return opened_; }
    [[nodiscard]] const SteamVrTrackerBridgeReport& LastReport() const noexcept { return last_report_; }

private:
    void ResetReport(const char* status);

    SteamVrTrackerBridgeConfig config_;
    bool opened_ = false;
    std::intptr_t socket_ = -1;
    std::uint64_t sequence_ = 0;
    SteamVrTrackerBridgeReport last_report_{};
};

} // namespace bt
