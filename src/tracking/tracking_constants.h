#pragma once

#include "core/types.h"

namespace bt::tracking_constants {

// Average adult foot fallback. BodyCalibration values override this whenever
// a calibrated model is available.
inline constexpr float kDefaultFootLengthM = 0.24f;

// Shared reprojection scale. Body solver defaults and body-state quality
// normalization must agree.
inline constexpr float kReprojectionErrorMaxPx = 45.0f;

// Reliability term thresholds. Missing evidence is absent evidence, not
// perfect 1.0 evidence.
inline constexpr float kReliabilityUsableThreshold = 0.15f;
inline constexpr float kReliabilityMinPresentConfidence = 0.05f;

// Monocular projection physical/numeric bounds.
inline constexpr float kMonocularMinDepthM = 0.30f;
inline constexpr float kMonocularMaxDepthM = 8.00f;
inline constexpr float kMonocularMinFovDeg = 30.0f;
inline constexpr float kMonocularMaxFovDeg = 130.0f;
inline constexpr float kMonocularMinBodyPixelHeight = 40.0f;

// Contact confidence thresholds.
inline constexpr float kFootMinContactConfidence = 0.35f;
inline constexpr float kKneeMinContactConfidence = 0.42f;

// Body-state confidence thresholds. These numbers are downstream-facing:
// UI/OSC visibility, telemetry labels, and tracker confidence all inherit this
// contract, so keep them named instead of reintroducing local literals.
inline constexpr float kVisibleConfidenceThreshold = 0.20f;
inline constexpr float kIdentityStableThreshold = 0.55f;
inline constexpr float kIdentityUncertainThreshold = 0.35f;

// Body-state role confidence factors.
inline constexpr float kIdentityRoleFactorMin = 0.55f;
inline constexpr float kIdentityRoleFactorMax = 1.00f;
inline constexpr float kCameraSeenUnusableFallbackFactor = 0.35f;
inline constexpr float kNoCameraEvidencePredictionFactor = 0.50f;

} // namespace bt::tracking_constants
