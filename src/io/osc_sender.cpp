#include "io/osc_sender.h"

#include "core/math.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>

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

static_assert(kOscTrackerRoleCount == kTrackerPoseCount,
    "OSC tracker role count must match tracker synthesis role count");

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#endif

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

std::string SocketErrorMessage(const char* action, int error_code) {
    return std::string(action) + ": socket_error=" + std::to_string(error_code);
}

Status SocketErrorStatusFromCode(const char* action, int error_code) {
    return Status::Error(StatusCode::DeviceUnavailable, SocketErrorMessage(action, error_code));
}

Status SocketErrorStatus(const char* action) {
    return SocketErrorStatusFromCode(action, SocketLastErrorCode());
}

const char* AddressFamilyName(int family) noexcept {
    switch (family) {
    case AF_INET: return "AF_INET";
    case AF_INET6: return "AF_INET6";
    case AF_UNSPEC: return "AF_UNSPEC";
    default: return "AF_OTHER";
    }
}

OscOpenAttemptState MakeResolveFailureAttempt(int gai) {
    OscOpenAttemptState attempt;
    attempt.action = "resolve";
    attempt.error_code = gai;
    attempt.detail = "OSC target address could not be resolved: getaddrinfo=" + std::to_string(gai);
    return attempt;
}

OscOpenAttemptState MakeSocketAttempt(int attempt_index, const addrinfo& ai) {
    OscOpenAttemptState attempt;
    attempt.attempt = attempt_index;
    attempt.action = "socket";
    attempt.address_family = AddressFamilyName(ai.ai_family);
    attempt.socket_type = ai.ai_socktype;
    attempt.protocol = ai.ai_protocol;
    return attempt;
}

std::string FormatOpenAttemptSummary(const std::vector<OscOpenAttemptState>& attempts) {
    if (attempts.empty()) {
        return "no address attempts";
    }
    std::string out;
    for (const auto& attempt : attempts) {
        if (!out.empty()) {
            out += "; ";
        }
        out += "#" + std::to_string(attempt.attempt) + " " + attempt.action;
        if (!attempt.address_family.empty()) {
            out += " " + attempt.address_family;
        }
        if (attempt.error_code != 0) {
            out += " error=" + std::to_string(attempt.error_code);
        }
        if (!attempt.detail.empty()) {
            out += " (" + attempt.detail + ")";
        }
    }
    return out;
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

void AppendPaddedString(std::vector<std::uint8_t>& out, const std::string& s) {
    out.insert(out.end(), s.begin(), s.end());
    out.push_back(0);
    while ((out.size() % 4) != 0) {
        out.push_back(0);
    }
}

void AppendFloat32Be(std::vector<std::uint8_t>& out, float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(float));
    out.push_back(static_cast<std::uint8_t>((bits >> 24) & 0xff));
    out.push_back(static_cast<std::uint8_t>((bits >> 16) & 0xff));
    out.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xff));
    out.push_back(static_cast<std::uint8_t>(bits & 0xff));
}

void EncodeOscVector3MessageInto(std::vector<std::uint8_t>& packet, const std::string& address, float x, float y, float z) {
    packet.clear();
    packet.reserve(address.size() + 24);
    AppendPaddedString(packet, address);
    AppendPaddedString(packet, ",fff");
    AppendFloat32Be(packet, x);
    AppendFloat32Be(packet, y);
    AppendFloat32Be(packet, z);
}

float Degrees(float radians) {
    return radians * 57.29577951308232f;
}

Vec3f QuatToEulerDegrees(const Quatf& q_raw) {
    const Quatf q = Normalize(q_raw);

    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float x = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    const float y = std::abs(sinp) >= 1.0f
        ? std::copysign(1.5707963267948966f, sinp)
        : std::asin(sinp);

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float z = std::atan2(siny_cosp, cosy_cosp);

    return Vec3f{Degrees(x), Degrees(y), Degrees(z)};
}

std::string TrackerAddress(int index, const char* field) {
    if (index <= 0) {
        return {};
    }
    return "/tracking/trackers/" + std::to_string(index) + "/" + field;
}

bool FiniteQuat(const Quatf& q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

float QuatLengthSq(const Quatf& q) {
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

bool QuatHasUsableLength(const Quatf& q) {
    if (!FiniteQuat(q)) {
        return false;
    }
    const float len_sq = QuatLengthSq(q);
    return std::isfinite(len_sq) && len_sq >= 0.25f && len_sq <= 4.0f;
}

float EffectiveOscMinConfidence(const OscConfig& config) {
    return std::clamp(config.min_confidence, 0.0f, 1.0f);
}

bool EvidenceUsableForOsc(const TrackerEvidence& evidence, const OscConfig&) {
    if (!evidence.valid) {
        return false;
    }
    switch (evidence.source) {
    case TrackerEvidenceSource::DirectStereo:
    case TrackerEvidenceSource::InferredMonocular:
    case TrackerEvidenceSource::ReplayInput:
    case TrackerEvidenceSource::AnchorHeld:
        return true;
    case TrackerEvidenceSource::HmdPrediction:
    case TrackerEvidenceSource::Predicted:
    case TrackerEvidenceSource::None:
    default:
        return false;
    }
}

bool EvidenceDegradedForOsc(const TrackerEvidence& evidence) {
    return evidence.degraded ||
        evidence.stereo_fallback ||
        evidence.source == TrackerEvidenceSource::AnchorHeld ||
        evidence.source == TrackerEvidenceSource::HmdPrediction ||
        evidence.source == TrackerEvidenceSource::Predicted;
}

std::string TrackerRejectionReason(const TrackerPose& tracker, const OscConfig& config) {
    if (!tracker.valid) {
        return "invalid";
    }
    if (tracker.confidence < EffectiveOscMinConfidence(config)) {
        return "below_min_confidence";
    }
    if (!IsFinite(tracker.pose.position)) {
        return "nonfinite_position";
    }
    if (!FiniteQuat(tracker.pose.orientation)) {
        return "nonfinite_orientation";
    }
    if (!QuatHasUsableLength(tracker.pose.orientation)) {
        return "invalid_orientation_length";
    }
    if (!EvidenceUsableForOsc(tracker.evidence, config)) {
        if (tracker.evidence.source == TrackerEvidenceSource::None) {
            return "no_tracker_evidence";
        }
        if (tracker.evidence.source == TrackerEvidenceSource::Predicted ||
            tracker.evidence.source == TrackerEvidenceSource::HmdPrediction) {
            return "predicted_only_tracker_evidence";
        }
        return "invalid_tracker_evidence";
    }
    return {};
}

OscSendReport MakeInitialReport(const OscConfig& config, bool opened) {
    OscSendReport report;
    report.enabled = config.enabled;
    report.open = opened;
    report.last_send_ok = true;
    report.status = config.enabled ? (opened ? "idle" : "closed") : "disabled";
    for (std::size_t i = 0; i < kTrackerRoles.size(); ++i) {
        const TrackerRole role = kTrackerRoles[i];
        auto& state = report.roles[i];
        state.role = role;
        state.tracker_index = DefaultVrchatTrackerIndex(role, config);
        state.configured = state.tracker_index > 0;
        state.reason = state.configured ? "not_sent" : "unmapped";
    }
    return report;
}

} // namespace

std::vector<std::uint8_t> EncodeOscVector3MessageForTest(const std::string& address, float x, float y, float z) {
    std::vector<std::uint8_t> packet;
    EncodeOscVector3MessageInto(packet, address, x, y, z);
    return packet;
}

Pose3f TransformCameraPoseToTrackerSpace(const Pose3f& camera_world_pose, const OscConfig& config) {
    const Quatf rotation = Normalize(config.tracker_space_rotation);
    Pose3f out;
    out.position = Add(
        Rotate(rotation, Scale(camera_world_pose.position, config.tracker_space_scale)),
        config.tracker_space_position_offset);
    out.orientation = Multiply(rotation, camera_world_pose.orientation);
    return out;
}

Pose3f TransformTrackerPoseToTrackerSpace(const TrackerPose& tracker, const OscConfig& config) {
    Pose3f out = TransformCameraPoseToTrackerSpace(tracker.pose, config);
    const std::size_t role_index = TrackerRoleIndex(tracker.role);
    if (role_index < config.tracker_space_role_offsets.size()) {
        out.position = Add(out.position, config.tracker_space_role_offsets[role_index]);
    }
    return out;
}

int DefaultVrchatTrackerIndex(TrackerRole role, const OscConfig& config) {
    switch (role) {
    case TrackerRole::Pelvis: return config.pelvis_tracker_index;
    case TrackerRole::LeftFoot: return config.left_foot_tracker_index;
    case TrackerRole::RightFoot: return config.right_foot_tracker_index;
    case TrackerRole::Chest: return config.chest_tracker_index;
    case TrackerRole::LeftElbow: return config.left_elbow_tracker_index;
    case TrackerRole::RightElbow: return config.right_elbow_tracker_index;
    case TrackerRole::LeftKnee: return config.left_knee_tracker_index;
    case TrackerRole::RightKnee: return config.right_knee_tracker_index;
    default: return -1;
    }
}

OscSender::OscSender(OscConfig config) : config_(std::move(config)) {
    RebuildAddressCache();
    last_report_ = MakeInitialReport(config_, opened_);
    packet_buffer_.reserve(96);
}

OscSender::~OscSender() {
    Close();
}

void OscSender::RebuildAddressCache() {
    for (const auto role : kTrackerRoles) {
        const std::size_t slot = TrackerRoleIndex(role);
        const int index = DefaultVrchatTrackerIndex(role, config_);
        position_addresses_[slot] = TrackerAddress(index, "position");
        rotation_addresses_[slot] = TrackerAddress(index, "rotation");
    }
}

const std::string& OscSender::PositionAddress(TrackerRole role) const {
    return position_addresses_[TrackerRoleIndex(role)];
}

const std::string& OscSender::RotationAddress(TrackerRole role) const {
    return rotation_addresses_[TrackerRoleIndex(role)];
}

void OscSender::SetSendMessageHookForTest(std::function<Status(const std::string&, float, float, float)> hook) {
    send_message_hook_for_test_ = std::move(hook);
}

Status OscSender::Open() {
    if (!config_.enabled) {
        opened_ = false;
        last_report_ = MakeInitialReport(config_, opened_);
        return Status::OK();
    }
    if (opened_) {
        return Status::OK();
    }
    if (const auto s = EnsureSocketsReady(); !s.ok()) {
        return s;
    }

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    addrinfo* results = nullptr;
    const std::string port = std::to_string(config_.target_port);
    const int gai = getaddrinfo(config_.target_address.c_str(), port.c_str(), &hints, &results);
    std::vector<OscOpenAttemptState> open_attempts;
    if (gai != 0 || !results) {
        open_attempts.push_back(MakeResolveFailureAttempt(gai));
        last_report_ = MakeInitialReport(config_, opened_);
        last_report_.last_send_ok = false;
        last_report_.status = "open_failed";
        last_report_.last_error = open_attempts.back().detail;
        last_report_.open_attempts = std::move(open_attempts);
        return Status::Error(StatusCode::DeviceUnavailable, last_report_.last_error);
    }

    Status last_error = Status::Error(StatusCode::DeviceUnavailable, "OSC socket connect failed: no address attempts");
    int attempt_index = 0;
    for (addrinfo* ai = results; ai != nullptr; ai = ai->ai_next) {
        OscOpenAttemptState attempt = MakeSocketAttempt(++attempt_index, *ai);
        const SocketHandle socket = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (socket == kInvalidSocket) {
            const int error_code = SocketLastErrorCode();
            attempt.error_code = error_code;
            attempt.detail = SocketErrorMessage("OSC socket() failed", error_code);
            last_error = SocketErrorStatusFromCode("OSC socket() failed", error_code);
            open_attempts.push_back(std::move(attempt));
            continue;
        }
        attempt.socket_created = true;
        attempt.action = "connect";
        if (::connect(socket, ai->ai_addr, static_cast<int>(ai->ai_addrlen)) == 0) {
            attempt.connected = true;
            attempt.detail = "connected";
            open_attempts.push_back(std::move(attempt));
            socket_ = FromSocket(socket);
            opened_ = true;
            last_report_ = MakeInitialReport(config_, opened_);
            last_report_.open_attempts = std::move(open_attempts);
            freeaddrinfo(results);
            return Status::OK();
        }
        const int error_code = SocketLastErrorCode();
        attempt.error_code = error_code;
        attempt.detail = SocketErrorMessage("OSC connect() failed", error_code);
        last_error = SocketErrorStatusFromCode("OSC connect() failed", error_code);
        open_attempts.push_back(std::move(attempt));
        CloseSocket(socket);
    }

    freeaddrinfo(results);
    last_report_ = MakeInitialReport(config_, opened_);
    last_report_.last_send_ok = false;
    last_report_.status = "open_failed";
    last_report_.open_attempts = std::move(open_attempts);
    last_report_.last_error = last_error.message + " attempts=" + FormatOpenAttemptSummary(last_report_.open_attempts);
    return Status::Error(last_error.code, last_report_.last_error);
}

void OscSender::Close() {
    const SocketHandle socket = ToSocket(socket_);
    if (socket != kInvalidSocket) {
        CloseSocket(socket);
    }
    socket_ = -1;
    opened_ = false;
    last_report_ = MakeInitialReport(config_, opened_);
}

Status OscSender::UpdateConfig(const OscConfig& next_config) {
    const bool transport_changed =
        config_.enabled != next_config.enabled ||
        config_.target_address != next_config.target_address ||
        config_.target_port != next_config.target_port;

    config_ = next_config;
    RebuildAddressCache();

    if (transport_changed) {
        if (opened_) {
            Close();
        }
        if (config_.enabled) {
            return Open();
        }
    } else {
        last_report_.enabled = config_.enabled;
    }
    return Status::OK();
}

Status OscSender::SendOscMessage(const std::string& address, float x, float y, float z) {
    if (send_message_hook_for_test_) {
        return send_message_hook_for_test_(address, x, y, z);
    }
    EncodeOscVector3MessageInto(packet_buffer_, address, x, y, z);
    const SocketHandle socket = ToSocket(socket_);
    if (socket == kInvalidSocket) {
        return Status::Error(StatusCode::FailedPrecondition, "OSC sender socket is invalid");
    }
#ifdef _WIN32
    const int sent = send(socket, reinterpret_cast<const char*>(packet_buffer_.data()), static_cast<int>(packet_buffer_.size()), 0);
#else
    const ssize_t sent = send(socket, packet_buffer_.data(), packet_buffer_.size(), 0);
#endif
    if (sent < 0) {
        return SocketErrorStatus("OSC UDP send failed");
    }
    if (static_cast<std::size_t>(sent) != packet_buffer_.size()) {
        return Status::Error(
            StatusCode::DeviceUnavailable,
            "OSC UDP send wrote partial packet: " + std::to_string(sent) +
                " of " + std::to_string(packet_buffer_.size()) + " bytes");
    }
    return Status::OK();
}

Status OscSender::SendTrackers(const TrackerPoseArray& trackers) {
    last_report_ = MakeInitialReport(config_, opened_);
    if (!config_.enabled) {
        last_report_.status = "disabled";
        return Status::OK();
    }
    if (!opened_) {
        last_report_.last_send_ok = false;
        last_report_.status = "not_open";
        last_report_.last_error = "OSC sender is not open";
        return Status::Error(StatusCode::FailedPrecondition, last_report_.last_error);
    }
    const bool base_transform_finite =
        IsFinite(config_.tracker_space_position_offset) &&
        QuatHasUsableLength(config_.tracker_space_rotation) &&
        std::isfinite(config_.tracker_space_scale) &&
        config_.tracker_space_scale > 0.0f;
    if (!config_.tracker_space_transform_valid || !base_transform_finite) {
        last_report_.last_send_ok = false;
        last_report_.status = "blocked_tracker_space";
        last_report_.last_error = config_.tracker_space_transform_valid
            ? "OSC tracker-space transform is not finite"
            : "OSC tracker-space transform is not calibrated";
        for (auto& role_report : last_report_.roles) {
            if (role_report.configured) {
                role_report.reason = "blocked_tracker_space";
                ++last_report_.skipped_tracker_count;
            }
        }
        return Status::Error(StatusCode::FailedPrecondition, last_report_.last_error);
    }
    const bool stale_tracker_space =
        config_.tracker_space_source == "steamvr_controller_alignment_stale";
    if (stale_tracker_space) {
        last_report_.status = "degraded_tracker_space";
        last_report_.last_error = "SteamVR tracker-space alignment is stale; sending last numeric transform";
    }

    bool transport_failed = false;
    bool invalid_mapping = false;
    for (std::size_t role_index = 0; role_index < kTrackerRoles.size(); ++role_index) {
        TrackerPose tracker = trackers[role_index];
        tracker.role = kTrackerRoles[role_index];

        auto& role_report = last_report_.roles[role_index];
        role_report.degraded = EvidenceDegradedForOsc(tracker.evidence) || stale_tracker_space;
        const std::string rejection = TrackerRejectionReason(tracker, config_);
        role_report.valid = rejection.empty();
        if (!role_report.configured) {
            role_report.reason = "unmapped";
            ++last_report_.skipped_tracker_count;
            continue;
        }
        if (!role_report.valid) {
            role_report.reason = rejection.empty() ? "invalid" : rejection;
            ++last_report_.skipped_tracker_count;
            continue;
        }
        const int index = DefaultVrchatTrackerIndex(tracker.role, config_);
        if (index < 1 || index > 8) {
            role_report.valid = false;
            role_report.reason = "invalid_index_out_of_vrchat_range";
            role_report.error_detail = "VRChat tracker indices must be in the range 1..8";
            last_report_.last_send_ok = false;
            last_report_.last_error = std::string(ToString(tracker.role)) + ": " + role_report.error_detail;
            invalid_mapping = true;
            ++last_report_.skipped_tracker_count;
            continue;
        }
        const Pose3f vr_pose = TransformTrackerPoseToTrackerSpace(tracker, config_);
        if (!IsFinite(vr_pose.position) || !QuatHasUsableLength(vr_pose.orientation)) {
            role_report.valid = false;
            role_report.reason = !IsFinite(vr_pose.position) ? "nonfinite_tracker_space_position" : "invalid_tracker_space_orientation";
            ++last_report_.skipped_tracker_count;
            continue;
        }
        if (const auto s = SendOscMessage(
                PositionAddress(tracker.role),
                vr_pose.position.x,
                vr_pose.position.y,
                vr_pose.position.z); !s.ok()) {
            role_report.reason = "transport_failed";
            role_report.error_detail = s.message;
            last_report_.last_send_ok = false;
            last_report_.last_error = std::string(ToString(tracker.role)) + " position: " + s.message;
            transport_failed = true;
            ++last_report_.skipped_tracker_count;
            continue;
        }
        ++last_report_.sent_message_count;
        if (config_.send_rotations) {
            const Vec3f euler = QuatToEulerDegrees(vr_pose.orientation);
            if (const auto s = SendOscMessage(RotationAddress(tracker.role), euler.x, euler.y, euler.z); !s.ok()) {
                role_report.reason = "transport_failed";
                role_report.error_detail = s.message;
                last_report_.last_send_ok = false;
                last_report_.last_error = std::string(ToString(tracker.role)) + " rotation: " + s.message;
                transport_failed = true;
                ++last_report_.skipped_tracker_count;
                continue;
            }
            ++last_report_.sent_message_count;
        }
        role_report.sent = true;
        role_report.reason = "sent";
        ++last_report_.sent_tracker_count;
    }

    if (last_report_.sent_tracker_count > 0) {
        const bool partial = !last_report_.last_send_ok || last_report_.skipped_tracker_count > 0;
        if (stale_tracker_space) {
            last_report_.status = partial
                ? "degraded_tracker_space_partial_sent"
                : "degraded_tracker_space_sent";
        } else {
            last_report_.status = partial ? "partial_sent" : "sent";
        }
        return Status::OK();
    }
    if (transport_failed) {
        last_report_.status = "send_failed";
        return Status::Error(StatusCode::DeviceUnavailable, last_report_.last_error);
    }
    if (invalid_mapping) {
        last_report_.status = "invalid_index";
        return Status::Error(StatusCode::ValidationError, last_report_.last_error);
    }
    last_report_.status = "no_trackers";
    return Status::OK();
}

} // namespace bt
