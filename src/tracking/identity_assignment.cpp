#include "tracking/identity_assignment.h"

#include "tracking/measurement_weighting.h"
#include "tracking/triangulation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace bt {

float IdentityAnisotropicMahalanobisSq(
    const Vec3f& measured_world,
    const Vec3f& expected_world,
    const Vec3f& depth_axis_world,
    float lateral_stddev_m,
    float depth_stddev_m) noexcept;

namespace {

struct LateralPair {
    KeypointId left;
    KeypointId right;
};

constexpr std::array<LateralPair, 3> kScoredPairs{{
    {KeypointId::LeftHip, KeypointId::RightHip},
    {KeypointId::LeftKnee, KeypointId::RightKnee},
    {KeypointId::LeftAnkle, KeypointId::RightAnkle},
}};

constexpr std::array<LateralPair, 11> kSwapPairs{{
    {KeypointId::LeftEye, KeypointId::RightEye},
    {KeypointId::LeftEar, KeypointId::RightEar},
    {KeypointId::LeftShoulder, KeypointId::RightShoulder},
    {KeypointId::LeftElbow, KeypointId::RightElbow},
    {KeypointId::LeftWrist, KeypointId::RightWrist},
    {KeypointId::LeftHip, KeypointId::RightHip},
    {KeypointId::LeftKnee, KeypointId::RightKnee},
    {KeypointId::LeftAnkle, KeypointId::RightAnkle},
    {KeypointId::LeftBigToe, KeypointId::RightBigToe},
    {KeypointId::LeftSmallToe, KeypointId::RightSmallToe},
    {KeypointId::LeftHeel, KeypointId::RightHeel},
}};

constexpr std::array<LateralPair, 6> kEpipolarIdentityPairs{{
    {KeypointId::LeftHip, KeypointId::RightHip},
    {KeypointId::LeftKnee, KeypointId::RightKnee},
    {KeypointId::LeftAnkle, KeypointId::RightAnkle},
    {KeypointId::LeftBigToe, KeypointId::RightBigToe},
    {KeypointId::LeftSmallToe, KeypointId::RightSmallToe},
    {KeypointId::LeftHeel, KeypointId::RightHeel},
}};

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

const Keypoint2D& Keypoint(const DecodedPose2D& pose, KeypointId id) {
    return pose.keypoints[static_cast<std::size_t>(id)];
}

bool Usable(const Keypoint2D& kp) {
    return kp.present && std::isfinite(kp.pixel.x) && std::isfinite(kp.pixel.y) && std::isfinite(kp.confidence) && kp.confidence > 0.0f;
}

struct CandidateScore {
    float score = 0.0f;
    float total_weight = 0.0f;
    float weighted_abs_separation = 0.0f;
    int usable_pairs = 0;
};

CandidateScore ScoreCandidate(const DecodedPose2D& observed, float expected_direction, bool swapped) {
    CandidateScore out;
    for (const auto pair : kScoredPairs) {
        const Keypoint2D& left = Keypoint(observed, swapped ? pair.right : pair.left);
        const Keypoint2D& right = Keypoint(observed, swapped ? pair.left : pair.right);
        if (!Usable(left) || !Usable(right)) {
            continue;
        }

        const float weight = Clamp01(std::min(left.confidence, right.confidence));
        const float separation = left.pixel.x - right.pixel.x;
        const float signed_separation = expected_direction * separation;
        const float disagreement = signed_separation < 0.0f ? -signed_separation : 0.0f;
        out.score += weight * disagreement;
        out.total_weight += weight;
        out.weighted_abs_separation += weight * std::abs(separation);
        out.usable_pairs += 1;
    }
    return out;
}

float Consistency(const CandidateScore& straight, const CandidateScore& swapped) {
    const float total_weight = std::max(straight.total_weight, swapped.total_weight);
    const int usable_pairs = std::max(straight.usable_pairs, swapped.usable_pairs);
    const float separation_sum = std::max(straight.weighted_abs_separation, swapped.weighted_abs_separation);
    if (total_weight <= 1e-5f || usable_pairs <= 0 || separation_sum <= 1e-5f) {
        return 0.0f;
    }

    const float decision_gap = std::abs(straight.score - swapped.score);
    const float decision_quality = Clamp01(decision_gap / separation_sum);
    const float coverage_quality = Clamp01(total_weight / static_cast<float>(kScoredPairs.size()));
    const float mean_separation = separation_sum / total_weight;
    const float separation_quality = Clamp01(mean_separation / (mean_separation + 16.0f));
    return Clamp01(decision_quality * coverage_quality * separation_quality);
}

void SwapKeypoints(DecodedPose2D& pose) {
    for (const auto pair : kSwapPairs) {
        std::swap(
            pose.keypoints[static_cast<std::size_t>(pair.left)],
            pose.keypoints[static_cast<std::size_t>(pair.right)]);
    }
}

bool ProjectX(const Mat34f& image_from_world, const Vec3f& world, float& out_x) {
    const float u =
        image_from_world.m[0] * world.x +
        image_from_world.m[1] * world.y +
        image_from_world.m[2] * world.z +
        image_from_world.m[3];
    const float w =
        image_from_world.m[8] * world.x +
        image_from_world.m[9] * world.y +
        image_from_world.m[10] * world.z +
        image_from_world.m[11];
    if (!std::isfinite(u) || !std::isfinite(w) || std::abs(w) <= 1e-5f) {
        return false;
    }
    out_x = u / w;
    return std::isfinite(out_x);
}

bool ExpectedDirectionFromProjection(
    const LowerBodyState& predicted_state,
    const Mat34f* image_from_world,
    float& expected_direction) {

    if (!image_from_world) {
        return false;
    }

    float left_x = 0.0f;
    float right_x = 0.0f;
    if (!ProjectX(*image_from_world, predicted_state.left_foot.position, left_x) ||
        !ProjectX(*image_from_world, predicted_state.right_foot.position, right_x)) {
        return false;
    }

    const float delta = left_x - right_x;
    if (!std::isfinite(delta) || std::abs(delta) <= 1e-5f) {
        return false;
    }

    expected_direction = delta > 0.0f ? 1.0f : -1.0f;
    return true;
}

bool ExpectedDirectionFromWorld(const LowerBodyState& predicted_state, float& expected_direction) {
    const float predicted_lateral_delta = predicted_state.left_foot.position.x - predicted_state.right_foot.position.x;
    if (!std::isfinite(predicted_lateral_delta) || std::abs(predicted_lateral_delta) <= 1e-5f) {
        return false;
    }
    expected_direction = predicted_lateral_delta > 0.0f ? 1.0f : -1.0f;
    return true;
}

IdentityAssignmentResult ResolveWithExpectedDirection(const DecodedPose2D& observed, float expected_direction) {
    IdentityAssignmentResult result;
    result.pose = observed;
    result.consistency = 0.0f;
    result.swapped = false;
    if (!observed.valid) {
        return result;
    }

    const CandidateScore straight = ScoreCandidate(observed, expected_direction, false);
    const CandidateScore swapped = ScoreCandidate(observed, expected_direction, true);
    result.consistency = Consistency(straight, swapped);
    if (swapped.score + 1e-4f < straight.score) {
        result.swapped = true;
        SwapKeypoints(result.pose);
    }
    return result;
}

IdentityAssignmentResult ForcedAssignment(const DecodedPose2D& observed, bool swapped, float consistency) {
    IdentityAssignmentResult result;
    result.pose = observed;
    result.consistency = Clamp01(consistency);
    result.swapped = swapped;
    if (observed.valid && swapped) {
        SwapKeypoints(result.pose);
    }
    if (!observed.valid) {
        result.consistency = 0.0f;
        result.swapped = false;
    }
    return result;
}

struct EpipolarAssignmentCandidate {
    // Geometry and detection are intentionally separate. Keypoint confidence is
    // used only as support/weight for observed correspondences; geometric
    // uncertainty comes from epipolar residuals and anisotropic 3-D residuals.
    float weighted_score_sum = 0.0f;
    float weighted_geometric_uncertainty_sum = 0.0f;
    float weighted_mahalanobis_sq_sum = 0.0f;
    float weighted_negative_log_likelihood_sum = 0.0f;
    float detection_support_sum = 0.0f;
    float weight_sum = 0.0f;
    int scored_lateral_pairs = 0;
    int uncertainty_fallback_count = 0;

    float score() const {
        if (weight_sum <= 1.0e-5f || scored_lateral_pairs <= 0) {
            return 0.0f;
        }
        return Clamp01(weighted_score_sum / weight_sum);
    }

    float geometric_uncertainty() const {
        if (weight_sum <= 1.0e-5f || scored_lateral_pairs <= 0) {
            return 1.0f;
        }
        return Clamp01(weighted_geometric_uncertainty_sum / weight_sum);
    }

    float mahalanobis_sq() const {
        if (weight_sum <= 1.0e-5f || scored_lateral_pairs <= 0) {
            return 0.0f;
        }
        return weighted_mahalanobis_sq_sum / weight_sum;
    }

    float negative_log_likelihood() const {
        if (weight_sum <= 1.0e-5f || scored_lateral_pairs <= 0) {
            return 0.0f;
        }
        return weighted_negative_log_likelihood_sum / weight_sum;
    }

    float detection_support() const {
        if (scored_lateral_pairs <= 0) {
            return 0.0f;
        }
        return Clamp01(detection_support_sum / static_cast<float>(2 * scored_lateral_pairs));
    }
};


StereoCameraModel IdentityCameraModelFromCalibration(const CameraCalibration& camera) {
    StereoCameraModel out;
    out.image_from_world = camera.image_from_world;
    out.world_from_camera = camera.world_from_camera;
    out.camera_matrix = camera.camera_matrix;
    out.projection_valid = camera.intrinsics_valid && camera.extrinsics_valid;
    return out;
}

Vec3f IdentityCameraOriginWorld(const CameraCalibration& camera) {
    return Vec3f{
        camera.world_from_camera.m[3],
        camera.world_from_camera.m[7],
        camera.world_from_camera.m[11]
    };
}

Vec3f IdentityProjectionDepthAxisWorld(const CameraCalibration& camera) {
    return NormalizeOr(
        Vec3f{
            camera.image_from_world.m[8],
            camera.image_from_world.m[9],
            camera.image_from_world.m[10]
        },
        Vec3f{0.0f, 0.0f, 1.0f});
}

Vec3f IdentityObservationDepthAxisWorld(
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b,
    const Vec3f& observed_world) {

    const Vec3f axis_a = SolverObservationDepthAxisFromOrigin(
        IdentityCameraOriginWorld(camera_a),
        observed_world,
        IdentityProjectionDepthAxisWorld(camera_a));
    const Vec3f axis_b = SolverObservationDepthAxisFromOrigin(
        IdentityCameraOriginWorld(camera_b),
        observed_world,
        IdentityProjectionDepthAxisWorld(camera_b));
    return NormalizeOr(Add(axis_a, axis_b), NormalizeOr(axis_a, Vec3f{0.0f, 0.0f, 1.0f}));
}

bool IsLeftIdentityKeypoint(KeypointId id) {
    switch (id) {
    case KeypointId::LeftHip:
    case KeypointId::LeftKnee:
    case KeypointId::LeftAnkle:
    case KeypointId::LeftBigToe:
    case KeypointId::LeftSmallToe:
    case KeypointId::LeftHeel:
        return true;
    default:
        return false;
    }
}

Vec3f IdentityReferenceWorld(const LowerBodyState& predicted_state, KeypointId id) {
    // Phase 6.15 identity arbitration has no full LowerBodyModel in this API.
    // Use the side-specific predicted foot/root anchor as an intentionally
    // uncertain state anchor, then combine that state covariance with the stereo
    // candidate measurement covariance below. This is not a perfect-pose claim.
    return IsLeftIdentityKeypoint(id)
        ? predicted_state.left_foot.position
        : predicted_state.right_foot.position;
}

const TrackerEvidence& IdentityReferenceEvidence(const LowerBodyState& predicted_state, KeypointId id) {
    return IsLeftIdentityKeypoint(id)
        ? predicted_state.left_foot_evidence
        : predicted_state.right_foot_evidence;
}

struct IdentityStatePriorStddev {
    float lateral_stddev_m = SolverObservationWeightingConfig{}.fallback_stddev_m;
    float depth_stddev_m = SolverObservationWeightingConfig{}.fallback_stddev_m;
    bool conservative_fallback = true;
};

float SafeIdentityStddev(float value, float fallback) {
    return std::isfinite(value) && value > 0.0f ? value : fallback;
}

float IdentityFallbackStddev(const StereoIdentityEpipolarContext& epipolar) {
    return SafeIdentityStddev(
        epipolar.solver_observation_weighting.fallback_stddev_m,
        SolverObservationWeightingConfig{}.fallback_stddev_m);
}

float EvidenceSupportConfidence(const TrackerEvidence& evidence, float state_confidence) {
    return Clamp01(std::max({evidence.direct_confidence, evidence.support_confidence, state_confidence}));
}

IdentityStatePriorStddev IdentityStatePriorStddevFor(
    const LowerBodyState& predicted_state,
    KeypointId id,
    const StereoIdentityEpipolarContext& epipolar) {

    IdentityStatePriorStddev out;
    const TrackerEvidence& evidence = IdentityReferenceEvidence(predicted_state, id);
    const float fallback_stddev_m = IdentityFallbackStddev(epipolar);
    const float process_stddev_m = SolverTemporalProcessStddevM(
        epipolar.predicted_state_age_seconds,
        epipolar.solver_observation_weighting);

    float base_lateral_stddev_m = fallback_stddev_m;
    float base_depth_stddev_m = fallback_stddev_m;
    if (evidence.valid &&
        !evidence.stale_aged &&
        evidence.source != TrackerEvidenceSource::None &&
        !evidence.stereo_fallback &&
        !evidence.degraded) {
        // The state anchor has no full covariance object yet. For valid,
        // current predicted-state evidence, use the configured state-anchor
        // reference stddevs as Sigma_predicted_state, then age them with process
        // noise below. These values replace the conservative fallback; they are
        // not added on top of candidate measurement covariance except through
        // the residual covariance sum in AddEpipolarPairScore().
        base_lateral_stddev_m = SafeIdentityStddev(
            epipolar.config.identity_prior_lateral_stddev_m,
            fallback_stddev_m);
        base_depth_stddev_m = SafeIdentityStddev(
            epipolar.config.identity_prior_depth_stddev_m,
            fallback_stddev_m);
        out.conservative_fallback = false;
    }

    const float support = EvidenceSupportConfidence(evidence, predicted_state.confidence);
    const float confidence_inflation = evidence.valid
        ? (1.0f + 2.0f * (1.0f - support))
        : 1.0f;

    base_lateral_stddev_m *= confidence_inflation;
    base_depth_stddev_m *= confidence_inflation;

    out.lateral_stddev_m = SolverStddevWithProcessNoise(
        base_lateral_stddev_m,
        process_stddev_m,
        epipolar.solver_observation_weighting);
    out.depth_stddev_m = SolverStddevWithProcessNoise(
        base_depth_stddev_m,
        process_stddev_m,
        epipolar.solver_observation_weighting);
    return out;
}


float IdentityDiagonalNegativeLogLikelihood(
    float mahalanobis_sq,
    float lateral_stddev_m,
    float depth_stddev_m) {

    const float lateral_var = std::max(1.0e-6f, lateral_stddev_m * lateral_stddev_m);
    const float depth_var = std::max(1.0e-6f, depth_stddev_m * depth_stddev_m);
    // For the diagonal covariance diag(lat, lat, depth), the NLL terms that can
    // differ between same/cross candidates are d^2 + log(det(Sigma)). The common
    // constants and 1/2 factor cancel in a same-vs-cross comparison.
    return mahalanobis_sq + std::log(lateral_var * lateral_var * depth_var);
}

float IdentityScoreFromMahalanobisSq(float mahalanobis_sq, const StereoIdentityEpipolarConfig& cfg) {
    if (!std::isfinite(mahalanobis_sq) || mahalanobis_sq < 0.0f) {
        return 0.0f;
    }
    const float scale = std::max(1.0f, cfg.identity_mahalanobis_score_scale);
    const float clipped = std::min(mahalanobis_sq, std::max(scale, cfg.identity_max_mahalanobis_sq));
    return Clamp01(1.0f / (1.0f + clipped / scale));
}

float CombinedStddevFromVarianceTerms(
    float measurement_stddev_m,
    float prior_stddev_m,
    float fallback_stddev_m) {

    const float measurement = SafeIdentityStddev(measurement_stddev_m, fallback_stddev_m);
    const float prior = SafeIdentityStddev(prior_stddev_m, fallback_stddev_m);
    return std::sqrt(measurement * measurement + prior * prior);
}

bool AddEpipolarPairScore(
    EpipolarAssignmentCandidate& candidate,
    const StereoIdentityEpipolarContext& epipolar,
    const LowerBodyState& predicted_state,
    KeypointId expected_id,
    const Keypoint2D& a,
    const Keypoint2D& b) {

    if (!Usable(a) || !Usable(b)) {
        return false;
    }

    const auto check = ComputeDistortionSafePixelSampsonEpipolarCheck(
        *epipolar.geometry,
        *epipolar.camera_a,
        *epipolar.camera_b,
        a.pixel,
        b.pixel,
        epipolar.config.soft_threshold_px,
        epipolar.config.hard_threshold_px);
    if (!check.valid) {
        return false;
    }

    const float weight = std::sqrt(Clamp01(a.confidence) * Clamp01(b.confidence));
    if (!std::isfinite(weight) || weight <= 0.0f) {
        return false;
    }

    const float epipolar_score = Clamp01(check.confidence);
    float directional_score = epipolar_score;
    float directional_uncertainty = 1.0f - epipolar_score;
    float mahalanobis_sq = 0.0f;
    float negative_log_likelihood = 0.0f;

    const auto tri = TriangulateLinearDLT(
        epipolar.camera_a->image_from_world,
        epipolar.camera_b->image_from_world,
        a.pixel,
        b.pixel,
        std::max(0.05f, Clamp01(a.confidence)),
        std::max(0.05f, Clamp01(b.confidence)),
        epipolar.triangulation);
    if (tri.ok()) {
        const StereoCameraModel camera_a_model = IdentityCameraModelFromCalibration(*epipolar.camera_a);
        const StereoCameraModel camera_b_model = IdentityCameraModelFromCalibration(*epipolar.camera_b);
        const auto uncertainty = EstimateStereoMeasurementUncertainty(
            camera_a_model,
            camera_b_model,
            tri.value(),
            check.sampson_error_px_anisotropic > 0.0f ? check.sampson_error_px_anisotropic : check.sampson_error_px,
            true,
            epipolar.uncertainty);

        const float fallback_stddev_m = IdentityFallbackStddev(epipolar);
        float measurement_lateral_stddev = fallback_stddev_m;
        float measurement_depth_stddev = fallback_stddev_m;
        if (uncertainty.valid &&
            uncertainty.unclamped_lateral_stddev_m > 0.0f &&
            uncertainty.unclamped_depth_stddev_m > 0.0f) {
            measurement_lateral_stddev = uncertainty.unclamped_lateral_stddev_m;
            measurement_depth_stddev = uncertainty.unclamped_depth_stddev_m;
        } else {
            candidate.uncertainty_fallback_count += 1;
        }

        // Identity matching compares one triangulated same/cross candidate
        // against the predicted side anchor. There is no independent second 3-D
        // observation in this API; the covariance of this residual is therefore:
        //   Sigma_difference = Sigma_candidate_measurement + Sigma_predicted_state.
        const IdentityStatePriorStddev state_prior =
            IdentityStatePriorStddevFor(predicted_state, expected_id, epipolar);
        if (state_prior.conservative_fallback) {
            candidate.uncertainty_fallback_count += 1;
        }
        const float combined_lateral_stddev = CombinedStddevFromVarianceTerms(
            measurement_lateral_stddev,
            state_prior.lateral_stddev_m,
            fallback_stddev_m);
        const float combined_depth_stddev = CombinedStddevFromVarianceTerms(
            measurement_depth_stddev,
            state_prior.depth_stddev_m,
            fallback_stddev_m);
        const Vec3f depth_axis_world = IdentityObservationDepthAxisWorld(
            *epipolar.camera_a,
            *epipolar.camera_b,
            tri.value().world);
        mahalanobis_sq = IdentityAnisotropicMahalanobisSq(
            tri.value().world,
            IdentityReferenceWorld(predicted_state, expected_id),
            depth_axis_world,
            combined_lateral_stddev,
            combined_depth_stddev);
        negative_log_likelihood = IdentityDiagonalNegativeLogLikelihood(
            mahalanobis_sq,
            combined_lateral_stddev,
            combined_depth_stddev);
        const float mahalanobis_score = IdentityScoreFromMahalanobisSq(mahalanobis_sq, epipolar.config);
        directional_score = epipolar_score * mahalanobis_score;
        directional_uncertainty = Clamp01(1.0f - directional_score);
    } else {
        // Missing 3-D uncertainty is explicitly conservative. It never becomes
        // zero covariance; the candidate keeps only epipolar support and is
        // marked so telemetry/review can see that fallback uncertainty was used.
        candidate.uncertainty_fallback_count += 1;
        directional_score = 0.25f * epipolar_score;
        directional_uncertainty = Clamp01(1.0f - directional_score);
        mahalanobis_sq = epipolar.config.identity_max_mahalanobis_sq;
        negative_log_likelihood = IdentityDiagonalNegativeLogLikelihood(
            mahalanobis_sq,
            IdentityFallbackStddev(epipolar),
            IdentityFallbackStddev(epipolar));
    }

    candidate.weighted_score_sum += weight * directional_score;
    candidate.weighted_geometric_uncertainty_sum += weight * directional_uncertainty;
    candidate.weighted_mahalanobis_sq_sum += weight * mahalanobis_sq;
    candidate.weighted_negative_log_likelihood_sum += weight * negative_log_likelihood;
    candidate.detection_support_sum += 0.5f * (Clamp01(a.confidence) + Clamp01(b.confidence));
    candidate.weight_sum += weight;
    return true;
}


EpipolarAssignmentCandidate ScoreEpipolarAssignment(
    const DecodedPose2D& pose_a,
    const DecodedPose2D& pose_b,
    const LowerBodyState& predicted_state,
    const StereoIdentityEpipolarContext& epipolar,
    bool cross_identity) {

    EpipolarAssignmentCandidate candidate;
    if (!epipolar.valid() || !pose_a.valid || !pose_b.valid) {
        return candidate;
    }

    for (const auto pair : kEpipolarIdentityPairs) {
        const Keypoint2D& a_left = Keypoint(pose_a, pair.left);
        const Keypoint2D& a_right = Keypoint(pose_a, pair.right);
        const Keypoint2D& b_left = Keypoint(pose_b, cross_identity ? pair.right : pair.left);
        const Keypoint2D& b_right = Keypoint(pose_b, cross_identity ? pair.left : pair.right);

        const bool scored_left = AddEpipolarPairScore(
            candidate,
            epipolar,
            predicted_state,
            pair.left,
            a_left,
            b_left);
        const bool scored_right = AddEpipolarPairScore(
            candidate,
            epipolar,
            predicted_state,
            pair.right,
            a_right,
            b_right);
        if (scored_left || scored_right) {
            candidate.scored_lateral_pairs += 1;
        }
    }

    return candidate;
}


void ApplyEpipolarIdentityArbitration(
    StereoIdentityAssignmentResult& out,
    const DecodedPose2D& observed_a,
    const DecodedPose2D& observed_b,
    const LowerBodyState& predicted_state,
    const StereoIdentityEpipolarContext& epipolar) {

    if (!epipolar.valid() || epipolar.degraded() || !out.camera_a.pose.valid || !out.camera_b.pose.valid) {
        return;
    }

    const EpipolarAssignmentCandidate same = ScoreEpipolarAssignment(out.camera_a.pose, out.camera_b.pose, predicted_state, epipolar, false);
    const EpipolarAssignmentCandidate cross = ScoreEpipolarAssignment(out.camera_a.pose, out.camera_b.pose, predicted_state, epipolar, true);
    out.epipolar_arbitration_checked = same.scored_lateral_pairs > 0 || cross.scored_lateral_pairs > 0;
    out.epipolar_scored_lateral_pairs = std::max(same.scored_lateral_pairs, cross.scored_lateral_pairs);
    out.epipolar_same_identity_score = same.score();
    out.epipolar_cross_identity_score = cross.score();
    out.epipolar_cross_geometric_uncertainty = cross.geometric_uncertainty();
    out.epipolar_detection_support = std::max(same.detection_support(), cross.detection_support());
    out.identity_same_mahalanobis_sq = same.mahalanobis_sq();
    out.identity_cross_mahalanobis_sq = cross.mahalanobis_sq();
    out.identity_same_negative_log_likelihood = same.negative_log_likelihood();
    out.identity_cross_negative_log_likelihood = cross.negative_log_likelihood();
    out.identity_uncertainty_fallback_count =
        same.uncertainty_fallback_count + cross.uncertainty_fallback_count;

    const auto& cfg = epipolar.config;
    if (out.epipolar_scored_lateral_pairs < std::max(1, cfg.min_scored_lateral_pairs)) {
        return;
    }
    if (out.epipolar_detection_support < cfg.min_detection_support) {
        return;
    }

    const float same_score = out.epipolar_same_identity_score;
    const float cross_score = out.epipolar_cross_identity_score;
    const float coverage = Clamp01(
        static_cast<float>(out.epipolar_scored_lateral_pairs) /
        static_cast<float>(std::max(1, cfg.min_scored_lateral_pairs)));
    const float required_absolute_margin =
        cfg.swap_absolute_margin +
        cfg.uncertainty_swap_margin_scale * out.epipolar_cross_geometric_uncertainty +
        cfg.partial_coverage_swap_margin * (1.0f - coverage);
    out.epipolar_required_swap_margin = required_absolute_margin;
    const float same_mahalanobis_sq = same.mahalanobis_sq();
    const float cross_mahalanobis_sq = cross.mahalanobis_sq();
    const float same_negative_log_likelihood = same.negative_log_likelihood();
    const float cross_negative_log_likelihood = cross.negative_log_likelihood();
    const bool cross_within_mahalanobis_gate =
        std::isfinite(cross_mahalanobis_sq) &&
        cross_mahalanobis_sq <= cfg.identity_max_mahalanobis_sq;
    out.identity_cross_within_mahalanobis_gate = cross_within_mahalanobis_gate;
    const bool likelihoods_comparable =
        std::isfinite(same_mahalanobis_sq) &&
        std::isfinite(same_negative_log_likelihood) &&
        std::isfinite(cross_negative_log_likelihood);
    const bool cross_score_strong_enough =
        cross_within_mahalanobis_gate &&
        cross_score >= cfg.min_assignment_score &&
        cross_score >= same_score * cfg.swap_ratio_margin &&
        (cross_score - same_score) >= required_absolute_margin;
    const bool cross_likelihood_strong_enough =
        cross_within_mahalanobis_gate &&
        likelihoods_comparable &&
        (same_negative_log_likelihood - cross_negative_log_likelihood) >=
            cfg.identity_swap_nll_margin;
    out.identity_score_gate_passed = cross_score_strong_enough;
    out.identity_likelihood_gate_passed = cross_likelihood_strong_enough;
    // A score-margin swap is still allowed for conservative policy continuity,
    // but it cannot bypass the anisotropic Mahalanobis hard gate. The
    // comparative path uses the likelihood form d^2 + log(det(Sigma)) so unequal
    // same/cross covariances do not masquerade as a raw d^2 improvement.
    if (!cross_score_strong_enough && !cross_likelihood_strong_enough) {
        return;
    }

    const bool a_guarded = out.camera_a.consistency >= cfg.strong_consistency_guard;
    const bool b_guarded = out.camera_b.consistency >= cfg.strong_consistency_guard;
    if (a_guarded && b_guarded) {
        out.identity_swap_blocked_by_strong_consistency = true;
        return;
    }

    constexpr float kTieBreakEpsilon = 0.03f;
    const bool a_weaker = out.camera_a.consistency + kTieBreakEpsilon < out.camera_b.consistency;
    const bool b_weaker = out.camera_b.consistency + kTieBreakEpsilon < out.camera_a.consistency;
    if (!a_weaker && !b_weaker) {
        out.identity_swap_blocked_by_tie = true;
        return;
    }

    const float arbitration_consistency = Clamp01(cross_score);
    if (a_weaker && !a_guarded) {
        out.camera_a = ForcedAssignment(observed_a, !out.camera_a.swapped, arbitration_consistency);
        out.cross_camera_override_applied = true;
        out.epipolar_arbitration_applied = true;
    } else if (b_weaker && !b_guarded) {
        out.camera_b = ForcedAssignment(observed_b, !out.camera_b.swapped, arbitration_consistency);
        out.cross_camera_override_applied = true;
        out.epipolar_arbitration_applied = true;
    }
}


} // namespace

float IdentityAnisotropicMahalanobisSq(
    const Vec3f& measured_world,
    const Vec3f& expected_world,
    const Vec3f& depth_axis_world,
    float lateral_stddev_m,
    float depth_stddev_m) noexcept {

    const Vec3f axis = NormalizeOr(depth_axis_world, Vec3f{0.0f, 0.0f, 1.0f});
    const Vec3f delta = Sub(measured_world, expected_world);
    const float depth_delta = Dot(delta, axis);
    const float total_sq = Dot(delta, delta);
    const float lateral_sq = std::max(0.0f, total_sq - depth_delta * depth_delta);
    const float lateral_var = std::max(1.0e-6f, lateral_stddev_m * lateral_stddev_m);
    const float depth_var = std::max(1.0e-6f, depth_stddev_m * depth_stddev_m);
    return lateral_sq / lateral_var + (depth_delta * depth_delta) / depth_var;
}

IdentityAssignmentResult ResolveLeftRightIdentity(const DecodedPose2D& observed, const LowerBodyState& predicted_state) {
    return ResolveLeftRightIdentity(observed, predicted_state, nullptr);
}

IdentityAssignmentResult ResolveLeftRightIdentity(
    const DecodedPose2D& observed,
    const LowerBodyState& predicted_state,
    const Mat34f* image_from_world) {

    float expected_direction = 0.0f;
    if (!ExpectedDirectionFromProjection(predicted_state, image_from_world, expected_direction) &&
        !ExpectedDirectionFromWorld(predicted_state, expected_direction)) {
        IdentityAssignmentResult result;
        result.pose = observed;
        result.consistency = 0.0f;
        result.swapped = false;
        return result;
    }

    return ResolveWithExpectedDirection(observed, expected_direction);
}

StereoIdentityAssignmentResult ResolveStereoLeftRightIdentity(
    const DecodedPose2D& observed_a,
    const DecodedPose2D& observed_b,
    const LowerBodyState& predicted_state,
    const Mat34f* image_from_world_a,
    const Mat34f* image_from_world_b,
    const StereoIdentityEpipolarContext* epipolar) {

    StereoIdentityAssignmentResult out;
    out.camera_a = ResolveLeftRightIdentity(observed_a, predicted_state, image_from_world_a);
    out.camera_b = ResolveLeftRightIdentity(observed_b, predicted_state, image_from_world_b);

    if (!observed_a.valid || !observed_b.valid) {
        return out;
    }

    if (out.camera_a.swapped == out.camera_b.swapped) {
        if (epipolar) {
            ApplyEpipolarIdentityArbitration(out, observed_a, observed_b, predicted_state, *epipolar);
        }
        return out;
    }

    // Cross-camera arbitration is intentionally conservative. A strong view may
    // rescue an ambiguous partner, but two confident views are left alone because
    // per-camera perspective can legitimately make the local 2-D evidence differ.
    constexpr float kStrongConsistency = 0.55f;
    constexpr float kWeakConsistency = 0.28f;
    constexpr float kOverrideGap = 0.22f;

    const bool a_stronger =
        out.camera_a.consistency >= kStrongConsistency &&
        out.camera_b.consistency <= kWeakConsistency &&
        (out.camera_a.consistency - out.camera_b.consistency) >= kOverrideGap;
    const bool b_stronger =
        out.camera_b.consistency >= kStrongConsistency &&
        out.camera_a.consistency <= kWeakConsistency &&
        (out.camera_b.consistency - out.camera_a.consistency) >= kOverrideGap;

    if (a_stronger) {
        out.camera_b = ForcedAssignment(observed_b, out.camera_a.swapped, out.camera_b.consistency);
        out.cross_camera_override_applied = true;
    } else if (b_stronger) {
        out.camera_a = ForcedAssignment(observed_a, out.camera_b.swapped, out.camera_a.consistency);
        out.cross_camera_override_applied = true;
    }

    if (epipolar) {
        ApplyEpipolarIdentityArbitration(out, observed_a, observed_b, predicted_state, *epipolar);
    }

    return out;
}

} // namespace bt
