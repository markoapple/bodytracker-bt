#pragma once

#include "core/config.h"
#include "core/status.h"
#include "tracking/tracker_synthesis.h"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace bt {

struct OscRoleSendState {
    TrackerRole role = TrackerRole::Pelvis;
    int tracker_index = 0;
    bool configured = false;
    bool valid = false;
    bool sent = false;
    bool degraded = false;
    std::string reason = "not_evaluated";
    std::string error_detail;
};

struct OscOpenAttemptState {
    int attempt = 0;
    std::string action;
    std::string address_family;
    int socket_type = 0;
    int protocol = 0;
    int error_code = 0;
    bool socket_created = false;
    bool connected = false;
    std::string detail;
};

struct OscSendReport {
    bool enabled = false;
    bool open = false;
    bool last_send_ok = true;
    std::string status = "disabled";
    std::string last_error;
    int sent_tracker_count = 0;
    int skipped_tracker_count = 0;
    int sent_message_count = 0;
    std::array<OscRoleSendState, kTrackerPoseCount> roles{};
    std::vector<OscOpenAttemptState> open_attempts{};
};

class OscSender {
public:
    explicit OscSender(OscConfig config);
    ~OscSender();

    OscSender(const OscSender&) = delete;
    OscSender& operator=(const OscSender&) = delete;

    Status Open();
    void Close();
    Status UpdateConfig(const OscConfig& next_config);
    Status SendTrackers(const TrackerPoseArray& trackers);
    [[nodiscard]] bool IsOpen() const noexcept { return opened_; }
    [[nodiscard]] const OscSendReport& LastReport() const noexcept { return last_report_; }
    [[nodiscard]] const std::array<std::string, kTrackerPoseCount>& PositionAddressesForTest() const noexcept { return position_addresses_; }
    [[nodiscard]] const std::array<std::string, kTrackerPoseCount>& RotationAddressesForTest() const noexcept { return rotation_addresses_; }
    void SetSendMessageHookForTest(std::function<Status(const std::string&, float, float, float)> hook);

private:
    Status SendOscMessage(const std::string& address, float x, float y, float z);
    void RebuildAddressCache();
    [[nodiscard]] const std::string& PositionAddress(TrackerRole role) const;
    [[nodiscard]] const std::string& RotationAddress(TrackerRole role) const;

    OscConfig config_;
    bool opened_ = false;
    std::intptr_t socket_ = -1;
    std::array<std::string, kTrackerPoseCount> position_addresses_{};
    std::array<std::string, kTrackerPoseCount> rotation_addresses_{};
    std::vector<std::uint8_t> packet_buffer_{};
    OscSendReport last_report_{};
    std::function<Status(const std::string&, float, float, float)> send_message_hook_for_test_{};
};

std::vector<std::uint8_t> EncodeOscVector3MessageForTest(const std::string& address, float x, float y, float z);
Pose3f TransformCameraPoseToTrackerSpace(const Pose3f& camera_world_pose, const OscConfig& config);
Pose3f TransformTrackerPoseToTrackerSpace(const TrackerPose& tracker, const OscConfig& config);
int DefaultVrchatTrackerIndex(TrackerRole role, const OscConfig& config);

} // namespace bt
