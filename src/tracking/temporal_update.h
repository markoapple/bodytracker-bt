#pragma once

#include "core/config.h"
#include "core/types.h"
#include "tracking/body_model.h"

namespace bt {

enum class TemporalPositionCorrectionMode {
    BlendPositions = 0,
    DirectMeasuredPositions
};

LowerBodyState PredictState(
    const LowerBodyState& previous,
    double dt_seconds,
    const LowerBodyModel* model = nullptr);
LowerBodyState CorrectState(
    const LowerBodyState& predicted,
    const LowerBodyState& measured,
    double dt_seconds,
    const TemporalUpdateConfig& config = {},
    TemporalPositionCorrectionMode position_mode = TemporalPositionCorrectionMode::BlendPositions,
    const LowerBodyModel* model = nullptr);

} // namespace bt
