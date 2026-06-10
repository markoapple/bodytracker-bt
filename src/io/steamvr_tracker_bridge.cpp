#include "io/steamvr_tracker_bridge.h"

#include "io/osc_sender.h"

#include <array>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <mutex>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace bt {
namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#endif

constexpr std::uint32_t kSteamVrBridgeMagic = 0x54535442u; // "BTST" little-endian.
constexpr std::uint16_t kSteamVrBridgeVersion = 1;

#pragma pack(push, 1)
struct BridgeRolePacket {
    std::uint8_t role = 0;
    std::uint8_t valid = 0;
    std::uint8_t degraded = 0;
    std::uint8_t reserved = 0;
    float confidence = 0.0f;
    float px = 0.0f;
    float py = 0.0f;
    float pz = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
};

struct BridgePacket {
    std::uint32_t magic = kSteamVrBridgeMagic;
    std::uint16_t version = kSteamVrBridgeVersion;
    std::uint16_t role_count = static_cast<std::uint16_t>(kTrackerPoseCount);
    std::uint64_t sequence = 0;
    double timestamp_seconds = 0.0;
    std::array<BridgeRolePacket, kTrackerPoseCount> roles{};
};
#pragma pack(pop)

SocketHandle ToSocket(std::intptr_t value) {
    return static_cast<SocketHandle>(value);
}

std::intptr_t FromSocket(SocketHandle socket) {
    return static_cast<std::intptr_t>(socket);
}

void CloseSocket(SocketHandle socket) {
    if (socket == kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

int SocketLastErrorCode() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

Status SocketErrorStatus(const char* action) {
    return Status::Error(StatusCode::DeviceUnavailable, std::string(action) + ": socket_error=" + std::to_string(SocketLastErrorCode()));
}

Status EnsureSocketsReady() {
#ifdef _WIN32
    static std::once_flag once;
    static Status result = Status::OK();
    std::call_once(once, []() {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            result = Status::Error(StatusCode::DeviceUnavailable, "WSAStartup failed");
        }
    });
    return result;
#else
    return Status::OK();
#endif
}

bool FiniteQuat(const Quatf& q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

bool QuatHasUsableLength(const Quatf& q) {
    if (!FiniteQuat(q)) {
        return false;
    }
    const float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(len_sq) && len_sq >= 0.25f && len_sq <= 4.0f;
}

bool TrackerSpaceFinite(const OscConfig& cfg) {
    return IsFinite(cfg.tracker_space_position_offset) &&
        QuatHasUsableLength(cfg.tracker_space_rotation) &&
        std::isfinite(cfg.tracker_space_scale) &&
        cfg.tracker_space_scale > 0.0f &&
        std::all_of(cfg.tracker_space_role_offsets.begin(), cfg.tracker_space_role_offsets.end(), [](const Vec3f& offset) {
            return IsFinite(offset);
        });
}


bool RoleEnabledForBridge(const SteamVrTrackerBridgeConfig& config, TrackerRole role) {
    switch (role) {
    case TrackerRole::Pelvis:
    case TrackerRole::LeftFoot:
    case TrackerRole::RightFoot:
        return true;
    case TrackerRole::Chest:
        return config.send_chest;
    case TrackerRole::LeftElbow:
    case TrackerRole::RightElbow:
        return config.send_elbows;
    case TrackerRole::LeftKnee:
    case TrackerRole::RightKnee:
        return config.send_knees;
    default:
        return false;
    }
}

bool TrackerUsableForSteamVr(const TrackerPose& tracker, float min_confidence) {
    return tracker.valid &&
        tracker.confidence >= std::clamp(min_confidence, 0.0f, 1.0f) &&
        IsFinite(tracker.pose.position) &&
        QuatHasUsableLength(tracker.pose.orientation) &&
        tracker.evidence.valid;
}

std::string SteamVrTrackerInvalidReason(const TrackerPose& tracker, float min_confidence) {
    if (!tracker.valid) {
        return "invalid_tracker_pose";
    }
    if (tracker.confidence < std::clamp(min_confidence, 0.0f, 1.0f)) {
        return "below_min_confidence";
    }
    if (!IsFinite(tracker.pose.position)) {
        return "nonfinite_position";
    }
    if (!QuatHasUsableLength(tracker.pose.orientation)) {
        return "invalid_orientation";
    }
    if (!tracker.evidence.valid) {
        return "invalid_tracker_evidence";
    }
    return "not_sent";
}

bool TrackerDegradedForSteamVr(const TrackerPose& tracker, const OscConfig& tracker_space) {
    return tracker.evidence.degraded ||
        tracker.evidence.stereo_fallback ||
        tracker.evidence.anchor_held ||
        tracker.evidence.source == TrackerEvidenceSource::AnchorHeld ||
        tracker.evidence.source == TrackerEvidenceSource::HmdPrediction ||
        tracker.evidence.source == TrackerEvidenceSource::Predicted ||
        tracker_space.tracker_space_source == "steamvr_controller_alignment_stale";
}

bool SameSteamVrTrackerBridgeConfig(const SteamVrTrackerBridgeConfig& a, const SteamVrTrackerBridgeConfig& b) {
    return a.enabled == b.enabled &&
        a.target_address == b.target_address &&
        a.target_port == b.target_port &&
        a.min_confidence == b.min_confidence &&
        a.send_chest == b.send_chest &&
        a.send_elbows == b.send_elbows &&
        a.send_knees == b.send_knees;
}

} // namespace

SteamVrTrackerBridgeSender::SteamVrTrackerBridgeSender(SteamVrTrackerBridgeConfig config)
    : config_(std::move(config)) {
    ResetReport(config_.enabled ? "closed" : "disabled");
}

SteamVrTrackerBridgeSender::~SteamVrTrackerBridgeSender() {
    Close();
}

void SteamVrTrackerBridgeSender::ResetReport(const char* status) {
    last_report_ = SteamVrTrackerBridgeReport{};
    last_report_.enabled = config_.enabled;
    last_report_.open = opened_;
    last_report_.status = status;
    last_report_.target_address = config_.target_address;
    last_report_.target_port = config_.target_port;
    last_report_.min_confidence = std::clamp(config_.min_confidence, 0.0f, 1.0f);
    last_report_.sequence = sequence_;
    for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
        const bool role_enabled = config_.enabled && RoleEnabledForBridge(config_, kTrackerRoles[i]);
        last_report_.role_enabled[i] = role_enabled;
        last_report_.role_valid[i] = false;
        last_report_.role_sent[i] = false;
        last_report_.role_degraded[i] = false;
        last_report_.role_confidence[i] = 0.0f;
        last_report_.role_reasons[i] = role_enabled ? "not_sent" : "disabled_by_config";
    }
}

Status SteamVrTrackerBridgeSender::Open() {
    ResetReport(config_.enabled ? "closed" : "disabled");
    if (!config_.enabled) {
        return Status::OK();
    }
    if (opened_) {
        ResetReport("idle");
        return Status::OK();
    }
    if (const auto s = EnsureSocketsReady(); !s.ok()) {
        last_report_.last_send_ok = false;
        last_report_.status = "open_failed";
        last_report_.last_error = s.message;
        return s;
    }

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    addrinfo* results = nullptr;
    const std::string port = std::to_string(config_.target_port);
    const int gai = getaddrinfo(config_.target_address.c_str(), port.c_str(), &hints, &results);
    if (gai != 0 || !results) {
        last_report_.last_send_ok = false;
        last_report_.status = "open_failed";
        last_report_.last_error = "SteamVR tracker bridge target address could not be resolved: getaddrinfo=" + std::to_string(gai);
        return Status::Error(StatusCode::DeviceUnavailable, last_report_.last_error);
    }

    Status last_error = Status::Error(StatusCode::DeviceUnavailable, "SteamVR tracker bridge UDP connect failed");
    for (addrinfo* ai = results; ai != nullptr; ai = ai->ai_next) {
        const SocketHandle socket = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (socket == kInvalidSocket) {
            last_error = SocketErrorStatus("SteamVR tracker bridge socket() failed");
            continue;
        }
        if (::connect(socket, ai->ai_addr, static_cast<int>(ai->ai_addrlen)) == 0) {
            socket_ = FromSocket(socket);
            opened_ = true;
            freeaddrinfo(results);
            ResetReport("idle");
            return Status::OK();
        }
        last_error = SocketErrorStatus("SteamVR tracker bridge connect() failed");
        CloseSocket(socket);
    }

    freeaddrinfo(results);
    last_report_.last_send_ok = false;
    last_report_.status = "open_failed";
    last_report_.last_error = last_error.message;
    return last_error;
}

void SteamVrTrackerBridgeSender::Close() {
    const SocketHandle socket = ToSocket(socket_);
    if (socket != kInvalidSocket) {
        CloseSocket(socket);
    }
    socket_ = -1;
    opened_ = false;
    ResetReport(config_.enabled ? "closed" : "disabled");
}

Status SteamVrTrackerBridgeSender::UpdateConfig(const SteamVrTrackerBridgeConfig& next_config) {
    if (SameSteamVrTrackerBridgeConfig(config_, next_config)) {
        return Status::OK();
    }
    const bool transport_changed =
        config_.enabled != next_config.enabled ||
        config_.target_address != next_config.target_address ||
        config_.target_port != next_config.target_port;
    config_ = next_config;
    if (transport_changed) {
        Close();
        if (config_.enabled) {
            return Open();
        }
    }
    ResetReport(config_.enabled ? (opened_ ? "idle" : "closed") : "disabled");
    return Status::OK();
}

Status SteamVrTrackerBridgeSender::SendTrackers(const TrackerPoseArray& trackers, const OscConfig& tracker_space, double timestamp_seconds) {
    ResetReport(config_.enabled ? "idle" : "disabled");
    if (!config_.enabled) {
        return Status::OK();
    }
    if (!opened_) {
        last_report_.last_send_ok = false;
        last_report_.status = "not_open";
        last_report_.last_error = "SteamVR tracker bridge sender is not open";
        return Status::Error(StatusCode::FailedPrecondition, last_report_.last_error);
    }
    if (!tracker_space.tracker_space_transform_valid || !TrackerSpaceFinite(tracker_space)) {
        last_report_.last_send_ok = false;
        last_report_.status = "blocked_tracker_space";
        last_report_.last_error = tracker_space.tracker_space_transform_valid
            ? "SteamVR tracker bridge tracker-space transform is not finite"
            : "SteamVR tracker bridge tracker-space transform is not calibrated";
        last_report_.skipped_tracker_count = static_cast<int>(kTrackerPoseCount);
        for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
            if (last_report_.role_enabled[i]) {
                last_report_.role_reasons[i] = "blocked_tracker_space";
            }
        }
        return Status::Error(StatusCode::FailedPrecondition, last_report_.last_error);
    }

    BridgePacket packet{};
    packet.sequence = ++sequence_;
    packet.timestamp_seconds = timestamp_seconds;

    for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
        TrackerPose tracker = trackers[i];
        tracker.role = kTrackerRoles[i];
        auto& role = packet.roles[i];
        role.role = static_cast<std::uint8_t>(i);
        role.confidence = std::clamp(tracker.confidence, 0.0f, 1.0f);
        last_report_.role_confidence[i] = role.confidence;
        if (!RoleEnabledForBridge(config_, kTrackerRoles[i])) {
            role.reserved = 1; // disabled by config: receiver should clear/disconnect this virtual tracker.
            last_report_.role_reasons[i] = "disabled_by_config";
            ++last_report_.skipped_tracker_count;
            continue;
        }
        last_report_.role_enabled[i] = true;
        if (!TrackerUsableForSteamVr(tracker, config_.min_confidence)) {
            last_report_.role_reasons[i] = SteamVrTrackerInvalidReason(tracker, config_.min_confidence);
            ++last_report_.skipped_tracker_count;
            continue;
        }
        last_report_.role_valid[i] = true;
        const Pose3f vr_pose = TransformTrackerPoseToTrackerSpace(tracker, tracker_space);
        if (!IsFinite(vr_pose.position) || !QuatHasUsableLength(vr_pose.orientation)) {
            last_report_.role_reasons[i] = "nonfinite_tracker_space_pose";
            ++last_report_.skipped_tracker_count;
            continue;
        }
        const Quatf q = Normalize(vr_pose.orientation);
        role.valid = 1;
        role.degraded = TrackerDegradedForSteamVr(tracker, tracker_space) ? 1 : 0;
        last_report_.role_degraded[i] = role.degraded != 0;
        role.px = vr_pose.position.x;
        role.py = vr_pose.position.y;
        role.pz = vr_pose.position.z;
        role.qx = q.x;
        role.qy = q.y;
        role.qz = q.z;
        role.qw = q.w;
        last_report_.role_sent[i] = true;
        last_report_.role_reasons[i] = last_report_.role_degraded[i] ? "sent_degraded" : "sent";
        ++last_report_.sent_tracker_count;
    }

    const SocketHandle socket = ToSocket(socket_);
    if (socket == kInvalidSocket) {
        last_report_.last_send_ok = false;
        last_report_.status = "not_open";
        last_report_.last_error = "SteamVR tracker bridge socket is invalid";
        return Status::Error(StatusCode::FailedPrecondition, last_report_.last_error);
    }
#ifdef _WIN32
    const int sent = send(socket, reinterpret_cast<const char*>(&packet), static_cast<int>(sizeof(packet)), 0);
#else
    const ssize_t sent = send(socket, &packet, sizeof(packet), 0);
#endif
    if (sent < 0) {
        const auto s = SocketErrorStatus("SteamVR tracker bridge UDP send failed");
        last_report_.last_send_ok = false;
        last_report_.status = "send_failed";
        last_report_.last_error = s.message;
        return s;
    }
    if (static_cast<std::size_t>(sent) != sizeof(packet)) {
        last_report_.last_send_ok = false;
        last_report_.status = "send_failed";
        last_report_.last_error = "SteamVR tracker bridge UDP send wrote partial packet";
        return Status::Error(StatusCode::DeviceUnavailable, last_report_.last_error);
    }

    last_report_.last_send_ok = true;
    last_report_.open = opened_;
    last_report_.sent_message_count = 1;
    last_report_.sequence = sequence_;
    last_report_.status = last_report_.sent_tracker_count > 0
        ? (last_report_.skipped_tracker_count > 0 ? "partial_sent" : "sent")
        : "all_trackers_invalid";
    return Status::OK();
}

} // namespace bt
