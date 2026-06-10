#pragma once

#include "core/math.h"
#include "core/types.h"

#include <array>
#include <chrono>
#include <functional>
#include <string>

namespace bt {

enum class SteamVrControllerRole {
    Unknown = 0,
    LeftHand,
    RightHand
};

inline const char* ToString(SteamVrControllerRole role) {
    switch (role) {
    case SteamVrControllerRole::LeftHand: return "left_hand";
    case SteamVrControllerRole::RightHand: return "right_hand";
    case SteamVrControllerRole::Unknown:
    default: return "unknown";
    }
}

struct SteamVrControllerPose {
    SteamVrControllerRole role = SteamVrControllerRole::Unknown;
    Pose3f pose{};
    bool valid = false;
    bool trigger_pressed = false;
    bool trigger_pressed_edge = false;
    double timestamp_seconds = 0.0;
    double pose_age_seconds = 0.0;
    std::string reason = "unavailable";
};

struct SteamVrPoseSnapshot {
    bool available = false;
    bool runtime_initialized = false;
    int device_count = 0;
    std::string status = "unavailable";
    std::string reason = "SteamVR provider not initialized";
    SteamVrControllerPose left{};
    SteamVrControllerPose right{};
    HmdPoseSample hmd{};
    bool hmd_tracked = false;
    std::string hmd_tracking_status = "unavailable";
};

// Abstract pose provider. Real OpenVR-backed implementation lives in steamvr_provider.cpp.
// The fake provider is a separate class used by tests; both expose the same Poll() shape.
class ISteamVrPoseProvider {
public:
    virtual ~ISteamVrPoseProvider() = default;
    [[nodiscard]] virtual SteamVrPoseSnapshot Poll() = 0;
};

class SteamVrPoseProvider : public ISteamVrPoseProvider {
public:
    SteamVrPoseProvider();
    ~SteamVrPoseProvider() override;

    SteamVrPoseProvider(const SteamVrPoseProvider&) = delete;
    SteamVrPoseProvider& operator=(const SteamVrPoseProvider&) = delete;

    [[nodiscard]] SteamVrPoseSnapshot Poll() override;
    [[nodiscard]] bool RuntimeInitialized() const noexcept { return vr_system_ != nullptr; }

private:
    static constexpr int kMaxTrackedDevices = 64;
    void* vr_system_ = nullptr;
    bool init_attempted_ = false;
    std::chrono::steady_clock::time_point next_init_attempt_{};
    std::string last_error_;
    std::array<bool, kMaxTrackedDevices> trigger_was_pressed_{};
    std::array<bool, kMaxTrackedDevices> pose_seen_{};
    std::array<double, kMaxTrackedDevices> last_valid_pose_time_seconds_{};
};

// Deterministic test/replay provider. Used by tests so SteamVR is not required for CI.
// The poll callback returns the next snapshot; tests inject one with whatever poses they need.
class FakeSteamVrPoseProvider : public ISteamVrPoseProvider {
public:
    using PollFn = std::function<SteamVrPoseSnapshot()>;

    FakeSteamVrPoseProvider() = default;
    explicit FakeSteamVrPoseProvider(SteamVrPoseSnapshot fixed) : fixed_(std::move(fixed)), use_fixed_(true) {}
    explicit FakeSteamVrPoseProvider(PollFn cb) : cb_(std::move(cb)) {}

    void SetSnapshot(SteamVrPoseSnapshot snapshot) {
        fixed_ = std::move(snapshot);
        use_fixed_ = true;
        cb_ = nullptr;
    }

    [[nodiscard]] SteamVrPoseSnapshot Poll() override {
        if (cb_) {
            return cb_();
        }
        return fixed_;
    }

private:
    SteamVrPoseSnapshot fixed_{};
    bool use_fixed_ = false;
    PollFn cb_{};
};

// Factory used by main.cpp/runtime: returns the real provider when OpenVR is compiled,
// otherwise a stub/null provider that always reports unavailable. Never throws.
SteamVrPoseSnapshot MakeUnavailableSnapshot(const char* reason);

} // namespace bt
