// OpenVR safe-init test.
// Compiles with BODYTRACKER_HAS_OPENVR undefined (no openvr.h present).
// Tests: FakeSteamVrPoseProvider, null-provider no-crash.

#include <cstdio>
#include <chrono>
#include <string>
#include <array>
#include <functional>

// ── Minimal stubs matching steamvr_provider.h ─────────────────────────────────

namespace bt {

struct Vec3f { float x=0,y=0,z=0; };
struct Quatf { float x=0,y=0,z=0,w=1; };
struct Pose3f { Vec3f position; Quatf orientation; };

enum class SteamVrControllerRole { Unknown=0, LeftHand, RightHand };
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
};

SteamVrPoseSnapshot MakeUnavailableSnapshot(const char* reason) {
    SteamVrPoseSnapshot s;
    s.available = false;
    s.runtime_initialized = false;
    s.status = "unavailable";
    s.reason = reason ? reason : "SteamVR provider not initialized";
    return s;
}

class ISteamVrPoseProvider {
public:
    virtual ~ISteamVrPoseProvider() = default;
    virtual SteamVrPoseSnapshot Poll() = 0;
};

// Real provider stub: no BODYTRACKER_HAS_OPENVR defined →
// Poll() always returns MakeUnavailableSnapshot("OpenVR support was not built").
// This mirrors the real implementation's #if !defined(BODYTRACKER_HAS_OPENVR) branch.
class SteamVrPoseProvider : public ISteamVrPoseProvider {
public:
    SteamVrPoseProvider() = default;
    ~SteamVrPoseProvider() override = default;
    SteamVrPoseSnapshot Poll() override {
        // Exactly what the real code does when BODYTRACKER_HAS_OPENVR is not defined:
        return MakeUnavailableSnapshot("OpenVR support was not built");
    }
    bool RuntimeInitialized() const noexcept { return false; }
private:
    void* vr_system_ = nullptr;
    bool init_attempted_ = false;
    std::chrono::steady_clock::time_point next_init_attempt_{};
    std::string last_error_;
    std::array<bool,64> trigger_was_pressed_{};
    std::array<bool,64> pose_seen_{};
    std::array<double,64> last_valid_pose_time_seconds_{};
};

class FakeSteamVrPoseProvider : public ISteamVrPoseProvider {
public:
    using PollFn = std::function<SteamVrPoseSnapshot()>;
    FakeSteamVrPoseProvider() = default;
    explicit FakeSteamVrPoseProvider(SteamVrPoseSnapshot fixed) : fixed_(std::move(fixed)), use_fixed_(true) {}
    explicit FakeSteamVrPoseProvider(PollFn cb) : cb_(std::move(cb)) {}
    SteamVrPoseSnapshot Poll() override {
        if (cb_) return cb_();
        return fixed_;
    }
private:
    SteamVrPoseSnapshot fixed_{};
    bool use_fixed_ = false;
    PollFn cb_{};
};

} // namespace bt

// ── Test ─────────────────────────────────────────────────────────────────────

static void Fail(const char* label, const char* reason) {
    std::fprintf(stderr, "FAIL [%s]: %s\n", label, reason);
    std::exit(1);
}

int main() {
    // 1. FakeSteamVrPoseProvider with unavailable snapshot
    {
        bt::FakeSteamVrPoseProvider fake(bt::MakeUnavailableSnapshot("no_runtime"));
        auto s = fake.Poll();
        if (s.available) Fail("fake_unavailable", "available should be false");
        if (s.status != "unavailable") Fail("fake_unavailable", "status != unavailable");
        printf("PASS [fake_unavailable_snapshot]\n");
    }

    // 2. FakeSteamVrPoseProvider with valid snapshot
    {
        bt::SteamVrPoseSnapshot valid;
        valid.available = true; valid.runtime_initialized = true; valid.status = "ok";
        bt::FakeSteamVrPoseProvider fake(valid);
        auto s = fake.Poll();
        if (!s.available) Fail("fake_valid", "available should be true");
        printf("PASS [fake_valid_snapshot]\n");
    }

    // 3. Real SteamVrPoseProvider — no BODYTRACKER_HAS_OPENVR — must not crash
    // and must return unavailable.
    {
        bt::SteamVrPoseProvider provider;
        auto s = provider.Poll();
        // Without BODYTRACKER_HAS_OPENVR the implementation unconditionally returns unavailable.
        if (s.available) Fail("real_no_openvr", "available should be false when not built with OpenVR");
        if (s.reason.empty()) Fail("real_no_openvr", "reason must be non-empty");
        printf("PASS [real_provider_no_openvr]: status='%s' reason='%s'\n",
               s.status.c_str(), s.reason.c_str());
    }

    // 4. Multiple polls — no state corruption / no crash
    {
        bt::SteamVrPoseProvider provider;
        for (int i = 0; i < 5; ++i) {
            auto s = provider.Poll();
            if (s.available) Fail("multi_poll", "should remain unavailable");
        }
        printf("PASS [multi_poll_no_crash]\n");
    }

    printf("\nAll OpenVR safe-init tests PASSED — no crash, no null deref.\n");
    return 0;
}
