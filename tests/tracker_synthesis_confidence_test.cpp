#include "tracking/tracker_synthesis.h"
#include "test_check.h"

namespace {

bt::FootSupportState Support(float confidence) {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = bt::FootSupportPhase::FlatPlant;
    support.anchor.active = true;
    support.anchor.confidence = confidence;
    return support;
}

} // namespace

int main() {
    bt::LowerBodyState state;
    state.confidence = 0.90f;

    auto trackers = bt::SynthesizeTrackerPoses(state);
    BT_CHECK_NEAR(trackers[1].confidence, 0.90f, 1e-5);
    BT_CHECK_NEAR(trackers[2].confidence, 0.90f, 1e-5);

    state.support.left_foot = Support(0.20f);
    state.support.right_foot = Support(0.95f);
    trackers = bt::SynthesizeTrackerPoses(state);

    BT_CHECK_NEAR(trackers[0].confidence, 0.90f, 1e-5);
    BT_CHECK_NEAR(trackers[1].confidence, 0.20f, 1e-5);
    BT_CHECK_NEAR(trackers[2].confidence, 0.95f, 1e-5);
    BT_CHECK(trackers[1].valid);
    BT_CHECK(trackers[2].valid);

    state.confidence = 0.10f;
    trackers = bt::SynthesizeTrackerPoses(state);
    BT_CHECK_NEAR(trackers[2].confidence, 0.95f, 1e-5);

    state.support.left_foot.anchor.confidence = 0.0f;
    trackers = bt::SynthesizeTrackerPoses(state);
    BT_CHECK_NEAR(trackers[1].confidence, 0.0f, 1e-5);
    BT_CHECK(!trackers[1].valid);

    return 0;
}
