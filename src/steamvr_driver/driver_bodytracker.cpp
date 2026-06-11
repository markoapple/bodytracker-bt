#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <openvr_driver.h>

#include "../io/bridge_protocol.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>

namespace {

using bt::bridge_protocol::BridgePacket;
using bt::bridge_protocol::BridgeRolePacket;
constexpr std::uint32_t kMagic = bt::bridge_protocol::kMagic;
constexpr std::uint16_t kVersion = bt::bridge_protocol::kVersion;
constexpr std::size_t kRoleCount = bt::bridge_protocol::kRoleCount;
constexpr int kListenPort = bt::bridge_protocol::kPort;
constexpr double kPoseTimeoutSeconds = 0.35;

struct RoleState {
    bool valid = false;
    bool degraded = false;
    float confidence = 0.0f;
    float px = 0.0f;
    float py = 0.0f;
    float pz = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    std::chrono::steady_clock::time_point last_update{};
};

std::mutex g_state_mutex;
std::array<RoleState, kRoleCount> g_roles{};
std::atomic<std::uint64_t> g_last_sequence{0};

const char* RoleName(std::size_t role) {
    switch (role) {
    case 0: return "Pelvis";
    case 1: return "Left Foot";
    case 2: return "Right Foot";
    case 3: return "Chest";
    case 4: return "Left Elbow";
    case 5: return "Right Elbow";
    case 6: return "Left Knee";
    case 7: return "Right Knee";
    default: return "Unknown";
    }
}

const char* RoleSerial(std::size_t role) {
    switch (role) {
    case 0: return "BT-Pelvis";
    case 1: return "BT-LeftFoot";
    case 2: return "BT-RightFoot";
    case 3: return "BT-Chest";
    case 4: return "BT-LeftElbow";
    case 5: return "BT-RightElbow";
    case 6: return "BT-LeftKnee";
    case 7: return "BT-RightKnee";
    default: return "BT-Unknown";
    }
}

const char* RoleControllerType(std::size_t role) {
    switch (role) {
    case 0: return "vive_tracker_waist";
    case 1: return "vive_tracker_left_foot";
    case 2: return "vive_tracker_right_foot";
    case 3: return "vive_tracker_chest";
    case 4: return "vive_tracker_left_elbow";
    case 5: return "vive_tracker_right_elbow";
    case 6: return "vive_tracker_left_knee";
    case 7: return "vive_tracker_right_knee";
    default: return "vive_tracker";
    }
}

const char* RoleInputProfile(std::size_t role) {
    switch (role) {
    case 0: return "{htc}/input/tracker/vive_tracker_waist_profile.json";
    case 1: return "{htc}/input/tracker/vive_tracker_left_foot_profile.json";
    case 2: return "{htc}/input/tracker/vive_tracker_right_foot_profile.json";
    case 3: return "{htc}/input/tracker/vive_tracker_chest_profile.json";
    case 4: return "{htc}/input/tracker/vive_tracker_left_elbow_profile.json";
    case 5: return "{htc}/input/tracker/vive_tracker_right_elbow_profile.json";
    case 6: return "{htc}/input/tracker/vive_tracker_left_knee_profile.json";
    case 7: return "{htc}/input/tracker/vive_tracker_right_knee_profile.json";
    default: return "{htc}/input/vive_tracker_profile.json";
    }
}

RoleState DefaultVisiblePoseForRole(std::size_t role) {
    RoleState out;
    out.valid = role <= 3; // waist, feet, and chest are the only defensible defaults.
    out.degraded = true;
    out.confidence = 0.01f;
    out.qw = 1.0f;
    out.last_update = std::chrono::steady_clock::now();
    switch (role) {
    case 0: out.px = 0.00f; out.py = 0.95f; out.pz = -0.25f; break; // waist/pelvis
    case 1: out.px = -0.13f; out.py = 0.08f; out.pz = -0.20f; break;
    case 2: out.px = 0.13f; out.py = 0.08f; out.pz = -0.20f; break;
    case 3: out.px = 0.00f; out.py = 1.35f; out.pz = -0.22f; break;
    case 4: out.px = -0.35f; out.py = 1.10f; out.pz = -0.20f; break;
    case 5: out.px = 0.35f; out.py = 1.10f; out.pz = -0.20f; break;
    case 6: out.px = -0.13f; out.py = 0.50f; out.pz = -0.20f; break;
    case 7: out.px = 0.13f; out.py = 0.50f; out.pz = -0.20f; break;
    default: break;
    }
    return out;
}

bool Finite(float v) {
    return std::isfinite(static_cast<double>(v));
}

bool RolePacketFinite(const BridgeRolePacket& r) {
    return Finite(r.px) && Finite(r.py) && Finite(r.pz) &&
        Finite(r.qx) && Finite(r.qy) && Finite(r.qz) && Finite(r.qw) &&
        Finite(r.confidence);
}

RoleState RoleStateFromPacket(const BridgeRolePacket& r, std::chrono::steady_clock::time_point now) {
    RoleState out;
    out.valid = r.valid != 0 && RolePacketFinite(r);
    out.degraded = r.degraded != 0;
    out.confidence = r.confidence;
    out.px = r.px;
    out.py = r.py;
    out.pz = r.pz;
    out.qx = r.qx;
    out.qy = r.qy;
    out.qz = r.qz;
    out.qw = r.qw;
    out.last_update = now;
    return out;
}

vr::DriverPose_t BasePose() {
    vr::DriverPose_t pose{};
    pose.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
    pose.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
    pose.qRotation = {1.0, 0.0, 0.0, 0.0};
    return pose;
}

vr::DriverPose_t PoseFromState(const RoleState& state, bool fresh) {
    vr::DriverPose_t pose = BasePose();
    if (!state.valid) {
        pose.result = vr::TrackingResult_Uninitialized;
        pose.poseIsValid = false;
        pose.deviceIsConnected = false;
        return pose;
    }

    pose.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
    pose.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
    pose.vecPosition[0] = state.px;
    pose.vecPosition[1] = state.py;
    pose.vecPosition[2] = state.pz;
    pose.qRotation = {state.qw, state.qx, state.qy, state.qz};
    pose.result = vr::TrackingResult_Running_OK;
    pose.poseIsValid = true;
    pose.willDriftInYaw = false;
    pose.shouldApplyHeadModel = false;
    pose.deviceIsConnected = true;
    return pose;
}

class BridgeUdpReceiver {
public:
    bool Open() {
#ifdef _WIN32
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            return false;
        }
        socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET) {
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(kListenPort);
        if (::bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
            return false;
        }
        u_long nonblocking = 1;
        ioctlsocket(socket_, FIONBIO, &nonblocking);
        return true;
#else
        return false;
#endif
    }

    void Close() {
#ifdef _WIN32
        if (socket_ != INVALID_SOCKET) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
        }
#endif
    }

    void Poll() {
#ifdef _WIN32
        if (socket_ == INVALID_SOCKET) {
            return;
        }
        for (int i = 0; i < 32; ++i) {
            BridgePacket packet{};
            const int got = recv(socket_, reinterpret_cast<char*>(&packet), sizeof(packet), 0);
            if (got == SOCKET_ERROR) {
                const int error = WSAGetLastError();
                if (error == WSAEWOULDBLOCK) {
                    break;
                }
                break;
            }
            if (got != static_cast<int>(sizeof(packet)) ||
                packet.magic != kMagic ||
                packet.version != kVersion ||
                packet.role_count != kRoleCount) {
                continue;
            }
            const auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(g_state_mutex);
                for (std::size_t role = 0; role < kRoleCount; ++role) {
                    if (packet.roles[role].reserved == 1) {
                        RoleState cleared;
                        cleared.valid = false;
                        cleared.last_update = now;
                        g_roles[role] = cleared;
                        continue;
                    }
                    const RoleState next = RoleStateFromPacket(packet.roles[role], now);
                    if (next.valid) {
                        g_roles[role] = next;
                    }
                }
            }
            g_last_sequence.store(packet.sequence, std::memory_order_relaxed);
        }
#endif
    }

private:
#ifdef _WIN32
    SOCKET socket_ = INVALID_SOCKET;
#endif
};

class BodyTrackerDevice final : public vr::ITrackedDeviceServerDriver {
public:
    explicit BodyTrackerDevice(std::size_t role) : role_(role) {}

    vr::EVRInitError Activate(std::uint32_t object_id) override {
        object_id_ = object_id;
        auto* props = vr::VRProperties();
        if (props) {
            const auto container = props->TrackedDeviceToPropertyContainer(object_id_);
            props->SetStringProperty(container, vr::Prop_TrackingSystemName_String, "bodytracker");
            props->SetStringProperty(container, vr::Prop_ManufacturerName_String, "bodytracker");
            props->SetStringProperty(container, vr::Prop_ModelNumber_String, RoleName(role_));
            props->SetStringProperty(container, vr::Prop_SerialNumber_String, RoleSerial(role_));
            props->SetStringProperty(container, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
            props->SetStringProperty(container, vr::Prop_RegisteredDeviceType_String, (std::string("bodytracker/") + RoleControllerType(role_) + "/" + RoleSerial(role_)).c_str());
            props->SetStringProperty(container, vr::Prop_InputProfilePath_String, RoleInputProfile(role_));
            props->SetStringProperty(container, vr::Prop_ControllerType_String, RoleControllerType(role_));
            props->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
            props->SetBoolProperty(container, vr::Prop_NeverTracked_Bool, false);
            props->SetBoolProperty(container, vr::Prop_WillDriftInYaw_Bool, false);
        }
        return vr::VRInitError_None;
    }

    void Deactivate() override {
        object_id_ = vr::k_unTrackedDeviceIndexInvalid;
    }

    void EnterStandby() override {}

    void* GetComponent(const char*) override {
        return nullptr;
    }

    void DebugRequest(const char* request, char* response_buffer, std::uint32_t response_buffer_size) override {
        if (!response_buffer || response_buffer_size == 0) {
            return;
        }
        const std::string response = std::string("bodytracker ") + RoleName(role_) +
            " seq=" + std::to_string(g_last_sequence.load(std::memory_order_relaxed)) +
            " request=" + (request ? request : "");
        strncpy_s(response_buffer, response_buffer_size, response.c_str(), _TRUNCATE);
    }

    vr::DriverPose_t GetPose() override {
        RoleState state;
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            state = g_roles[role_];
        }
        const auto now = std::chrono::steady_clock::now();
        const double age = state.last_update.time_since_epoch().count() == 0
            ? 999.0
            : std::chrono::duration<double>(now - state.last_update).count();
        return PoseFromState(state, age <= kPoseTimeoutSeconds);
    }

    void PublishPose() {
        if (object_id_ == vr::k_unTrackedDeviceIndexInvalid) {
            return;
        }
        if (auto* host = vr::VRServerDriverHost()) {
            const auto pose = GetPose();
            host->TrackedDevicePoseUpdated(object_id_, pose, sizeof(vr::DriverPose_t));
        }
    }

private:
    std::size_t role_ = 0;
    std::uint32_t object_id_ = vr::k_unTrackedDeviceIndexInvalid;
};

class BodyTrackerProvider final : public vr::IServerTrackedDeviceProvider {
public:
    BodyTrackerProvider() {
        for (std::size_t i = 0; i < kRoleCount; ++i) {
            devices_[i] = new BodyTrackerDevice(i);
        }
    }

    ~BodyTrackerProvider() {
        for (auto*& device : devices_) {
            delete device;
            device = nullptr;
        }
    }

    vr::EVRInitError Init(vr::IVRDriverContext* context) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(context);
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            for (std::size_t i = 0; i < kRoleCount; ++i) {
                g_roles[i] = DefaultVisiblePoseForRole(i);
            }
        }
        receiver_.Open();
        if (auto* host = vr::VRServerDriverHost()) {
            for (std::size_t i = 0; i < kRoleCount; ++i) {
                host->TrackedDeviceAdded(RoleSerial(i), vr::TrackedDeviceClass_GenericTracker, devices_[i]);
            }
        }
        return vr::VRInitError_None;
    }

    void Cleanup() override {
        receiver_.Close();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }

    const char* const* GetInterfaceVersions() override {
        return vr::k_InterfaceVersions;
    }

    void RunFrame() override {
        receiver_.Poll();
        for (auto* device : devices_) {
            if (device) {
                device->PublishPose();
            }
        }
    }

    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}

private:
    BridgeUdpReceiver receiver_;
    std::array<BodyTrackerDevice*, kRoleCount> devices_{};
};

BodyTrackerProvider g_provider;

} // namespace

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* interface_name, int* return_code) {
    if (return_code) {
        *return_code = vr::VRInitError_None;
    }
    if (interface_name && std::strcmp(interface_name, vr::IServerTrackedDeviceProvider_Version) == 0) {
        return &g_provider;
    }
    if (return_code) {
        *return_code = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}
