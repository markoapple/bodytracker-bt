#include "tracking/tracker_synthesis.h"
#include "test_check.h"

#include <limits>

namespace {

bt::TrackerEvidence Evidence(bt::TrackerEvidenceSource source, float confidence) {
    bt::TrackerEvidence evidence;
    evidence.source = source;
    evidence.direct_confidence = confidence;
    evidence.valid = confidence > 0.0f;
    return evidence;
}

} // namespace

int main() {
    bt::LowerBodyState state;
    state.confidence = 0.90f;
    state.root.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    state.left_foot.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    state.right_foot.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    state.left_foot_evidence = Evidence(bt::TrackerEvidenceSource::DirectStereo, 0.90f);
    state.right_foot_evidence = Evidence(bt::TrackerEvidenceSource::DirectStereo, 0.90f);

    state.left_foot_evidence.stale_aged = true;
    state.right_foot_evidence.degraded = true;
    auto trackers = bt::SynthesizeTrackerPoses(state);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].valid);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].valid);
    BT_CHECK_NEAR(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].confidence, 0.38f, 1e-5f);
    BT_CHECK_NEAR(trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].confidence, 0.62f, 1e-5f);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].evidence.stale_aged);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].evidence.degraded);

    state.left_foot.position.x = std::numeric_limits<float>::infinity();
    trackers = bt::SynthesizeTrackerPoses(state);
    BT_CHECK(!trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].valid);
    BT_CHECK_NEAR(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].confidence, 0.0f, 1e-6f);
    BT_CHECK(!trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].evidence.valid);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].valid);

    bt::LowerBodyState anchored;
    anchored.confidence = 0.95f;
    anchored.left_foot.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    anchored.left_foot_evidence.source = bt::TrackerEvidenceSource::AnchorHeld;
    anchored.left_foot_evidence.valid = true;
    anchored.left_foot_evidence.anchor_held = true;
    anchored.left_foot_evidence.support_confidence = 0.96f;
    trackers = bt::SynthesizeTrackerPoses(anchored);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].valid);
    BT_CHECK_NEAR(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].confidence, 0.96f, 1e-5f);

    return 0;
}
