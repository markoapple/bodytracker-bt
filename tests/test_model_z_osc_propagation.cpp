// Standalone falsification test: model Z changes monocular OSC z.
// No OpenCV, no ONNX, no external deps.
//
// Simulates the exact logic path:
//   DecodedPose3D (model z) → BuildMonocularSeeds (ApplyModelDepthToMonocularSeeds)
//   → seed.world.z → OSC candidate z
//
// Pass condition: two frames with different left-ankle model_z produce
//   different final OSC candidate z values.

#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <array>
#include <vector>
#include <cassert>
#include <string>

// ── Minimal type stubs (no opencv) ───────────────────────────────────────────

namespace bt {

constexpr std::size_t kHalpe26Count = 26;

enum class KeypointId : unsigned char {
    LeftAnkle = 15,
    Pelvis    = 19,
};

// From rtmpose_model_contract.h:
constexpr int kRtmw3dSimccZBins = 576;  // kRtmPoseHalpe26InputWidth(288) * 2

struct Vec3f { float x=0,y=0,z=0; };

struct ModelDepthKeypoint {
    float raw_z_bin     = 0.0f;
    float refined_z     = 0.0f;
    float z_logit       = 0.0f;
    float confidence_3d = 0.0f;
    bool  z_decoded     = false;
};

struct DecodedPose3D {
    bool valid = false;
    std::string coordinate_frame = "model_simcc_body_relative";
    std::array<ModelDepthKeypoint, kHalpe26Count> model_depth{};
};

// Fake kSolverKeypoints (only what we need for the test)
// In real code this is a compile-time array. Stub it here.
constexpr std::array<KeypointId, 2> kSolverKeypoints_test = {
    KeypointId::LeftAnkle,
    KeypointId::Pelvis,
};

static std::size_t SolverKeypointIndex(KeypointId id) {
    return static_cast<std::size_t>(id);
}

// Minimal WeightedJointSeed
struct WeightedJointSeed {
    Vec3f world{};
    float weight = 0.0f;
    bool  valid  = false;
};

using Seeds = std::array<WeightedJointSeed, kHalpe26Count>;

// ── Exact logic copied from ApplyModelDepthToMonocularSeeds ──────────────────

void ApplyModelDepthToMonocularSeeds(
    Seeds& seeds,
    const DecodedPose3D& pose_3d,
    float user_height_m)
{
    if (!pose_3d.valid) { return; }
    if (!std::isfinite(user_height_m) || user_height_m <= 0.1f) { return; }

    constexpr std::size_t kPelvisIdx = static_cast<std::size_t>(KeypointId::Pelvis);
    const auto& pelvis_dep = pose_3d.model_depth[kPelvisIdx];
    if (!pelvis_dep.z_decoded || pelvis_dep.refined_z <= 0.0f) { return; }

    constexpr float kZBinsHalf  = static_cast<float>(kRtmw3dSimccZBins) * 0.5f;
    constexpr float kMaxOffsetM = 1.5f;

    for (auto id : kSolverKeypoints_test) {
        const std::size_t i = SolverKeypointIndex(id);
        if (!seeds[i].valid) { continue; }
        const auto& dep = pose_3d.model_depth[i];
        if (!dep.z_decoded) { continue; }

        const float bin_offset  = dep.refined_z - pelvis_dep.refined_z;
        const float norm_offset = bin_offset / kZBinsHalf;
        const float z_offset_m  = norm_offset * user_height_m;
        if (!std::isfinite(z_offset_m)) { continue; }

        const float clamped = std::max(-kMaxOffsetM, std::min(kMaxOffsetM, z_offset_m));
        constexpr float kModelZBlend = 0.35f;
        seeds[i].world.z += kModelZBlend * clamped;
    }
}

// ── OSC transform stub (identity scale=1, no rotation) ───────────────────────

Vec3f TransformToTrackerSpace(const Vec3f& world) {
    // In real code: rotation + scale + offset. Identity here for test clarity.
    return world;
}

} // namespace bt

// ── Test ─────────────────────────────────────────────────────────────────────

int main() {
    using namespace bt;

    constexpr std::size_t kLA     = static_cast<std::size_t>(KeypointId::LeftAnkle);
    constexpr std::size_t kPelvis = static_cast<std::size_t>(KeypointId::Pelvis);

    const float user_height_m  = 1.70f;
    const float base_world_z   = 2.20f;  // typical monocular depth estimate
    const float pelvis_bin     = 288.0f; // center bin = reference

    // Helper: build seeds + pose_3d for a given ankle z bin, run fix, return OSC z
    auto run_frame = [&](float ankle_z_bin, const char* label) -> float {
        // Seed: left ankle and pelvis have valid world positions
        Seeds seeds{};
        seeds[kLA].world     = {0.0f, 0.05f, base_world_z};
        seeds[kLA].weight    = 0.85f;
        seeds[kLA].valid     = true;
        seeds[kPelvis].world = {0.0f, 0.90f, base_world_z};
        seeds[kPelvis].weight= 0.85f;
        seeds[kPelvis].valid = true;

        // pose_3d: pelvis at center bin, ankle at ankle_z_bin
        DecodedPose3D pose_3d;
        pose_3d.valid = true;
        pose_3d.model_depth[kPelvis].z_decoded  = true;
        pose_3d.model_depth[kPelvis].refined_z  = pelvis_bin;
        pose_3d.model_depth[kLA].z_decoded       = true;
        pose_3d.model_depth[kLA].refined_z       = ankle_z_bin;

        ApplyModelDepthToMonocularSeeds(seeds, pose_3d, user_height_m);

        const Vec3f osc = TransformToTrackerSpace(seeds[kLA].world);
        std::printf("  [%s] ankle_z_bin=%.1f → seed.world.z=%.4f → osc.z=%.4f\n",
                    label, ankle_z_bin, seeds[kLA].world.z, osc.z);
        return osc.z;
    };

    printf("=== Falsification test: model Z changes OSC z ===\n");

    // Frame A: ankle in front of pelvis (smaller bin = nearer)
    const float z_a = run_frame(188.0f, "frame_A_near");
    // Frame B: ankle behind pelvis (larger bin = farther)
    const float z_b = run_frame(388.0f, "frame_B_far");

    const float delta = std::abs(z_b - z_a);
    printf("  |osc_z_B - osc_z_A| = %.4f m\n", delta);

    // Expected: (388-288)/288 * 1.70 * 0.35 = (100/288) * 1.70 * 0.35 ≈ 0.207 m per step
    //           (188-288)/288 * 1.70 * 0.35 = (-100/288) * 1.70 * 0.35 ≈ -0.207 m
    //           total delta ≈ 0.414 m
    const float expected_min_delta = 0.30f;

    if (delta < expected_min_delta) {
        fprintf(stderr, "FAIL: delta %.4f < %.4f — model Z does NOT change OSC z\n",
                delta, expected_min_delta);
        return 1;
    }

    // Also verify z_a < z_b (nearer ankle → smaller world.z = closer depth)
    if (z_a >= z_b) {
        fprintf(stderr, "FAIL: expected z_a(near) < z_b(far), got z_a=%.4f z_b=%.4f\n", z_a, z_b);
        return 1;
    }

    // Frame C: model z absent (pose_3d.valid=false) — world.z must NOT change
    printf("\n=== Regression: no model z → world.z unchanged ===\n");
    {
        Seeds seeds{};
        seeds[kLA].world  = {0.0f, 0.05f, base_world_z};
        seeds[kLA].weight = 0.85f;
        seeds[kLA].valid  = true;

        DecodedPose3D no_depth;
        no_depth.valid = false;

        ApplyModelDepthToMonocularSeeds(seeds, no_depth, user_height_m);
        printf("  pose_3d.valid=false → seed.world.z=%.4f (expected %.4f)\n",
               seeds[kLA].world.z, base_world_z);
        if (std::abs(seeds[kLA].world.z - base_world_z) > 1e-6f) {
            fprintf(stderr, "FAIL: world.z changed when pose_3d invalid\n");
            return 1;
        }
    }

    // Frame D: same z bin for both ankle and pelvis → zero offset
    printf("\n=== Regression: same z bin as pelvis → no offset ===\n");
    {
        Seeds seeds{};
        seeds[kLA].world  = {0.0f, 0.05f, base_world_z};
        seeds[kLA].weight = 0.85f;
        seeds[kLA].valid  = true;
        seeds[kPelvis].world  = {0.0f, 0.90f, base_world_z};
        seeds[kPelvis].weight = 0.85f;
        seeds[kPelvis].valid  = true;

        DecodedPose3D same_depth;
        same_depth.valid = true;
        same_depth.model_depth[kPelvis].z_decoded = true;
        same_depth.model_depth[kPelvis].refined_z = 288.0f;
        same_depth.model_depth[kLA].z_decoded = true;
        same_depth.model_depth[kLA].refined_z = 288.0f;  // same as pelvis

        ApplyModelDepthToMonocularSeeds(seeds, same_depth, user_height_m);
        printf("  same bin → seed.world.z=%.4f (expected %.4f)\n",
               seeds[kLA].world.z, base_world_z);
        if (std::abs(seeds[kLA].world.z - base_world_z) > 1e-6f) {
            fprintf(stderr, "FAIL: world.z changed when ankle bin == pelvis bin\n");
            return 1;
        }
    }

    printf("\nPASS: model Z changes final OSC z by %.4f m (threshold %.2f m)\n",
           delta, expected_min_delta);
    printf("PASS: regressions clean\n");
    printf("\nFirst downstream lossy stage (BEFORE this fix): body_solver_input\n");
    printf("Reason: BuildMonocularSeeds ignored camera_a_pose_3d entirely.\n");
    printf("Fix: ApplyModelDepthToMonocularSeeds wires model z as depth offset.\n");
    return 0;
}
