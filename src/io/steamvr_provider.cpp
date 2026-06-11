#include "io/steamvr_provider.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>

#if defined(BODYTRACKER_HAS_OPENVR)
#include <openvr.h>
#include <stdexcept>
#if defined(_MSC_VER)
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif
#endif

namespace bt {
namespace {

#if defined(BODYTRACKER_HAS_OPENVR)
std::mutex& OpenVrProcessMutex() {
    static std::mutex mutex;
    return mutex;
}

vr::IVRSystem*& SharedOpenVrSystem() {
    static vr::IVRSystem* system = nullptr;
    return system;
}

double NowSeconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

Quatf QuatFromOpenVrMatrix(const vr::HmdMatrix34_t& m) {
    const Vec3f right{m.m[0][0], m.m[1][0], m.m[2][0]};
    const Vec3f up{m.m[0][1], m.m[1][1], m.m[2][1]};
    const Vec3f forward{m.m[0][2], m.m[1][2], m.m[2][2]};
    return QuatFromBasis(right, up, forward);
}

Pose3f PoseFromOpenVrMatrix(const vr::HmdMatrix34_t& m) {
    Pose3f out;
    out.position = Vec3f{m.m[0][3], m.m[1][3], m.m[2][3]};
    out.orientation = QuatFromOpenVrMatrix(m);
    return out;
}

bool ControllerTriggerPressed(vr::IVRSystem* system, vr::TrackedDeviceIndex_t device) {
    vr::VRControllerState_t state{};
#if defined(_MSC_VER)
    bool got_state = false;
    unsigned long seh_code = 0;
    __try {
        got_state = system->GetControllerState(device, &state, sizeof(state));
    } __except (seh_code = GetExceptionCode(), EXCEPTION_EXECUTE_HANDLER) {
        (void)seh_code;
        return false;
    }
    if (!got_state) {
        return false;
    }
#else
    if (!system->GetControllerState(device, &state, sizeof(state))) {
        return false;
    }
#endif
    const uint64_t trigger_button = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger);
    if ((state.ulButtonPressed & trigger_button) != 0) {
        return true;
    }
    constexpr int trigger_axis = static_cast<int>(vr::k_eControllerAxis_Trigger);
    if constexpr (trigger_axis >= 0 && trigger_axis < static_cast<int>(vr::k_unControllerStateAxisCount)) {
        return state.rAxis[trigger_axis].x > 0.75f;
    } else {
        return false;
    }
}

#if defined(_MSC_VER)
vr::IVRSystem* SafeVrInit(vr::EVRInitError& error, unsigned long& seh_code) {
    vr::IVRSystem* system = nullptr;
    seh_code = 0;
    __try {
        system = vr::VR_Init(&error, vr::VRApplication_Background);
    } __except (seh_code = GetExceptionCode(), EXCEPTION_EXECUTE_HANDLER) {
        system = nullptr;
        error = vr::VRInitError_Init_Internal;
    }
    return system;
}

bool SafeGetDeviceToAbsoluteTrackingPose(
    vr::IVRSystem* system,
    vr::TrackedDevicePose_t* poses,
    uint32_t pose_count,
    unsigned long& seh_code) {
    seh_code = 0;
    __try {
        system->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding, 0.0f, poses, pose_count);
    } __except (seh_code = GetExceptionCode(), EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
    return true;
}

bool SafeDeviceState(
    vr::IVRSystem* system,
    vr::TrackedDeviceIndex_t device,
    bool& connected,
    vr::ETrackedDeviceClass& device_class,
    vr::ETrackedControllerRole& role,
    unsigned long& seh_code) {
    seh_code = 0;
    __try {
        connected = system->IsTrackedDeviceConnected(device);
        device_class = connected ? system->GetTrackedDeviceClass(device) : vr::TrackedDeviceClass_Invalid;
        role = (connected && device_class == vr::TrackedDeviceClass_Controller)
            ? system->GetControllerRoleForTrackedDeviceIndex(device)
            : vr::TrackedControllerRole_Invalid;
    } __except (seh_code = GetExceptionCode(), EXCEPTION_EXECUTE_HANDLER) {
        connected = false;
        device_class = vr::TrackedDeviceClass_Invalid;
        role = vr::TrackedControllerRole_Invalid;
        return false;
    }
    return true;
}

void SafeVrShutdown() noexcept {
    __try {
        vr::VR_Shutdown();
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}
#endif
#endif

} // namespace

SteamVrPoseSnapshot MakeUnavailableSnapshot(const char* reason) {
    SteamVrPoseSnapshot s;
    s.available = false;
    s.runtime_initialized = false;
    s.device_count = 0;
    s.status = "unavailable";
    s.reason = reason ? reason : "SteamVR provider not initialized";
    return s;
}

SteamVrPoseProvider::SteamVrPoseProvider() = default;

SteamVrPoseProvider::~SteamVrPoseProvider() {
#if defined(BODYTRACKER_HAS_OPENVR)
    // OpenVR is process-global. The desktop runtime and the menu calibration
    // wizard both poll it, so individual provider destruction must not call
    // VR_Shutdown and invalidate the other user's IVRSystem pointer.
    vr_system_ = nullptr;
#endif
}

SteamVrPoseSnapshot SteamVrPoseProvider::Poll() {
#if !defined(BODYTRACKER_HAS_OPENVR)
    return MakeUnavailableSnapshot("OpenVR support was not built");
#else
    std::scoped_lock openvr_lock(OpenVrProcessMutex());
    SteamVrPoseSnapshot snapshot;
    auto& shared_system = SharedOpenVrSystem();
    if (shared_system) {
        vr_system_ = shared_system;
    }
    if (!vr_system_) {
        const auto now_clock = std::chrono::steady_clock::now();
        if (!init_attempted_ || now_clock >= next_init_attempt_) {
            init_attempted_ = true;
            vr::EVRInitError error = vr::VRInitError_None;
            try {
#if defined(_MSC_VER)
                unsigned long seh_code = 0;
                vr_system_ = SafeVrInit(error, seh_code);
                if (seh_code != 0) {
                    last_error_ = "VR_Init crashed inside OpenVR runtime (SEH 0x" +
                        std::to_string(static_cast<unsigned long long>(seh_code)) + "); OpenVR disabled until retry";
                }
#else
                vr_system_ = vr::VR_Init(&error, vr::VRApplication_Background);
#endif
            } catch (const std::exception& ex) {
                last_error_ = std::string("VR_Init threw: ") + ex.what();
                vr_system_  = nullptr;
                error        = vr::VRInitError_Init_VRClientDLLNotFound;
            } catch (...) {
                last_error_ = "VR_Init threw unknown exception";
                vr_system_  = nullptr;
                error        = vr::VRInitError_Init_VRClientDLLNotFound;
            }
            if (error == vr::VRInitError_None && vr_system_) {
                shared_system = static_cast<vr::IVRSystem*>(vr_system_);
                last_error_.clear();
            } else {
                if (last_error_.empty()) {
                    try {
                        last_error_ = vr::VR_GetVRInitErrorAsEnglishDescription(error);
                    } catch (...) {
                        last_error_ = "VR_Init failed (error description unavailable)";
                    }
                }
                vr_system_         = nullptr;
                shared_system      = nullptr;
                next_init_attempt_ = now_clock + std::chrono::seconds(2);
            }
        }
    }
    if (!vr_system_) {
        snapshot.available         = false;
        snapshot.runtime_initialized = false;
        snapshot.status            = "unavailable";
        snapshot.reason            = last_error_.empty() ? "SteamVR unavailable" : last_error_;
        return snapshot;
    }

    auto* system = static_cast<vr::IVRSystem*>(vr_system_);
    // GetDeviceToAbsoluteTrackingPose can also throw on a degraded runtime.
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    try {
#if defined(_MSC_VER)
        unsigned long seh_code = 0;
        if (!SafeGetDeviceToAbsoluteTrackingPose(system, poses, vr::k_unMaxTrackedDeviceCount, seh_code)) {
            SharedOpenVrSystem() = nullptr;
            vr_system_         = nullptr;
            last_error_        = "OpenVR pose query crashed (SEH 0x" +
                std::to_string(static_cast<unsigned long long>(seh_code)) + "); runtime reset";
            next_init_attempt_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            snapshot.available         = false;
            snapshot.runtime_initialized = false;
            snapshot.status            = "degraded";
            snapshot.reason            = last_error_;
            return snapshot;
        }
#else
        system->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);
#endif
    } catch (...) {
        SharedOpenVrSystem() = nullptr;
        vr_system_         = nullptr;
        last_error_        = "OpenVR pose query threw; runtime reset";
        next_init_attempt_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        snapshot.available         = false;
        snapshot.runtime_initialized = false;
        snapshot.status            = "degraded";
        snapshot.reason            = last_error_;
        return snapshot;
    }

    const double now = NowSeconds();
    snapshot.available = true;
    snapshot.runtime_initialized = true;
    snapshot.status = "connected";
    snapshot.reason = "SteamVR connected";

    const auto& hmd_pose = poses[vr::k_unTrackedDeviceIndex_Hmd];
    snapshot.hmd.pose = PoseFromOpenVrMatrix(hmd_pose.mDeviceToAbsoluteTracking);
    snapshot.hmd.timestamp_seconds = now;
    snapshot.hmd.valid = hmd_pose.bPoseIsValid && hmd_pose.eTrackingResult == vr::TrackingResult_Running_OK;
    snapshot.hmd_tracked = snapshot.hmd.valid;
    snapshot.hmd_tracking_status = snapshot.hmd.valid ? "tracked" : "hmd pose invalid";

    int controller_count = 0;
    bool any_controller_tracked = false;
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
        bool connected = false;
        vr::ETrackedDeviceClass device_class = vr::TrackedDeviceClass_Invalid;
        vr::ETrackedControllerRole role = vr::TrackedControllerRole_Invalid;
#if defined(_MSC_VER)
        unsigned long seh_code = 0;
        if (!SafeDeviceState(system, i, connected, device_class, role, seh_code)) {
            SharedOpenVrSystem() = nullptr;
            vr_system_         = nullptr;
            last_error_        = "OpenVR device query crashed (SEH 0x" +
                std::to_string(static_cast<unsigned long long>(seh_code)) + "); runtime reset";
            next_init_attempt_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            return MakeUnavailableSnapshot(last_error_.c_str());
        }
#else
        connected = system->IsTrackedDeviceConnected(i);
        device_class = connected ? system->GetTrackedDeviceClass(i) : vr::TrackedDeviceClass_Invalid;
        role = (connected && device_class == vr::TrackedDeviceClass_Controller)
            ? system->GetControllerRoleForTrackedDeviceIndex(i)
            : vr::TrackedControllerRole_Invalid;
#endif
        if (!connected || device_class != vr::TrackedDeviceClass_Controller) {
            continue;
        }
        ++controller_count;
        SteamVrControllerPose controller;
        controller.role = SteamVrControllerRole::Unknown;
        if (role == vr::TrackedControllerRole_LeftHand) {
            controller.role = SteamVrControllerRole::LeftHand;
        } else if (role == vr::TrackedControllerRole_RightHand) {
            controller.role = SteamVrControllerRole::RightHand;
        }
        controller.valid = poses[i].bPoseIsValid && poses[i].eTrackingResult == vr::TrackingResult_Running_OK;
        controller.pose = PoseFromOpenVrMatrix(poses[i].mDeviceToAbsoluteTracking);
        controller.trigger_pressed = ControllerTriggerPressed(system, i);
        const auto trigger_slot = static_cast<size_t>(i);
        const bool was_pressed = trigger_slot < trigger_was_pressed_.size() ? trigger_was_pressed_[trigger_slot] : false;
        controller.trigger_pressed_edge = controller.trigger_pressed && !was_pressed;
        if (trigger_slot < trigger_was_pressed_.size()) {
            trigger_was_pressed_[trigger_slot] = controller.trigger_pressed;
        }
        controller.timestamp_seconds = now;
        const double provider_offset_age = 0.0;
        if (controller.valid) {
            controller.pose_age_seconds = provider_offset_age;
            if (trigger_slot < pose_seen_.size()) {
                pose_seen_[trigger_slot] = true;
                last_valid_pose_time_seconds_[trigger_slot] = now;
            }
        } else if (trigger_slot < pose_seen_.size() && pose_seen_[trigger_slot]) {
            controller.pose_age_seconds = std::max(provider_offset_age, now - last_valid_pose_time_seconds_[trigger_slot]);
        } else {
            controller.pose_age_seconds = provider_offset_age;
        }
        controller.reason = controller.valid ? "tracked" : "controller pose invalid";
        if (controller.valid) {
            any_controller_tracked = true;
        }
        if (controller.role == SteamVrControllerRole::LeftHand) {
            snapshot.left = controller;
        } else if (controller.role == SteamVrControllerRole::RightHand) {
            snapshot.right = controller;
        }
    }
    snapshot.device_count = controller_count;
    if (controller_count == 0) {
        snapshot.status = "controllers_missing";
        snapshot.reason = "SteamVR connected but no controllers found";
    } else if (!any_controller_tracked) {
        snapshot.status = "controllers_untracked";
        snapshot.reason = "controllers present but not tracked";
    }
    return snapshot;
#endif
}

} // namespace bt
