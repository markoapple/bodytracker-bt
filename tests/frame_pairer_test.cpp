#include "capture/frame_pairer.h"
#include "test_check.h"

#include <memory>

namespace {

std::shared_ptr<bt::FramePacket> Frame(std::uint64_t sequence, std::int64_t ticks) {
    auto frame = std::make_shared<bt::FramePacket>();
    frame->sequence = sequence;
    frame->timestamp = bt::QpcTimestamp{ticks};
    return frame;
}

} // namespace

int main() {
    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a1 = Frame(1, 1'000'000'000);
        const auto b1 = Frame(1, 1'010'000'000);
        const auto first = pairer.Pair(a1, b1);
        BT_CHECK(first.valid);
        BT_CHECK(first.reason == "paired");
        BT_CHECK(pairer.Telemetry().accepted_pairs == 1);
        BT_CHECK(pairer.Telemetry().last_accepted_sequence_a == 1);
        BT_CHECK(pairer.Telemetry().last_accepted_sequence_b == 1);

        const auto a2 = Frame(2, 1'014'000'000);
        const auto b2 = Frame(2, 1'012'000'000);
        const auto fresh = pairer.Pair(a2, b2);
        BT_CHECK(fresh.valid);
        BT_CHECK(fresh.reason == "paired");
        BT_CHECK(pairer.Telemetry().accepted_pairs == 2);
        BT_CHECK(pairer.Telemetry().last_accepted_sequence_a == 2);
        BT_CHECK(pairer.Telemetry().last_accepted_sequence_b == 2);
    }

    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a1 = Frame(1, 1'000'000'000);
        const auto b1 = Frame(1, 1'010'000'000);
        BT_CHECK(pairer.Pair(a1, b1).valid);
        const auto duplicate = pairer.Pair(a1, b1);
        BT_CHECK(duplicate.valid);
        BT_CHECK(duplicate.reason == "duplicate_pair_degraded");
        BT_CHECK(pairer.Telemetry().degraded_duplicate == 1);
        BT_CHECK(pairer.Telemetry().rejected_duplicate == 0);
        BT_CHECK(pairer.Telemetry().accepted_pairs == 2);
    }

    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a1 = Frame(1, 1'000'000'000);
        const auto b1 = Frame(1, 1'010'000'000);
        BT_CHECK(pairer.Pair(a1, b1).valid);
        const auto b2 = Frame(2, 1'012'000'000);
        const auto reused_a = pairer.Pair(a1, b2);
        BT_CHECK(reused_a.valid);
        BT_CHECK(reused_a.reason == "reused_camera_a_frame_degraded");
        BT_CHECK(pairer.Telemetry().degraded_reused_a == 1);
        BT_CHECK(pairer.Telemetry().rejected_reused_a == 0);
        BT_CHECK(pairer.Telemetry().accepted_pairs == 2);
    }

    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a1 = Frame(1, 1'000'000'000);
        const auto b1 = Frame(1, 1'010'000'000);
        BT_CHECK(pairer.Pair(a1, b1).valid);
        const auto a2 = Frame(2, 1'014'000'000);
        const auto reused_b = pairer.Pair(a2, b1);
        BT_CHECK(reused_b.valid);
        BT_CHECK(reused_b.reason == "reused_camera_b_frame_degraded");
        BT_CHECK(pairer.Telemetry().degraded_reused_b == 1);
        BT_CHECK(pairer.Telemetry().rejected_reused_b == 0);
        BT_CHECK(pairer.Telemetry().accepted_pairs == 2);
    }

    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a3 = Frame(3, 2'000'000'000);
        const auto b3 = Frame(3, 2'050'000'000);
        const auto skewed = pairer.Pair(a3, b3);
        BT_CHECK(skewed.valid);
        BT_CHECK(skewed.reason == "timestamp_skew_degraded");
        BT_CHECK(pairer.Telemetry().degraded_skew == 1);
        BT_CHECK(pairer.Telemetry().rejected_skew == 0);
        BT_CHECK(pairer.Telemetry().accepted_pairs == 1);
    }

    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a3 = Frame(3, 2'000'000'000);
        const auto b4 = Frame(4, 2'010'000'000);
        const auto missing_a = pairer.Pair(nullptr, b4);
        BT_CHECK(!missing_a.valid);
        BT_CHECK(missing_a.reason == "missing_camera_a");
        BT_CHECK(pairer.Telemetry().missing_a == 1);

        const auto missing_b = pairer.Pair(a3, nullptr);
        BT_CHECK(!missing_b.valid);
        BT_CHECK(missing_b.reason == "missing_camera_b");
        BT_CHECK(pairer.Telemetry().missing_b == 1);
    }


    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a1 = Frame(1, 1'000'000'000);
        const auto b1 = Frame(1, 1'008'000'000);
        BT_CHECK(pairer.Pair(a1, b1).valid);

        const auto a2 = Frame(2, 1'016'000'000);
        const auto b2 = Frame(2, 1'001'000'000);
        const auto mixed = pairer.PairRecent({a1, a2}, {b2});
        BT_CHECK(mixed.valid);
        BT_CHECK(!mixed.degraded);
        BT_CHECK(!mixed.reused_a);
        BT_CHECK(!mixed.skewed);
        BT_CHECK(mixed.camera_a->sequence == 2);
        BT_CHECK(mixed.camera_b->sequence == 2);
        BT_CHECK(mixed.reason == "paired");
    }

    {
        bt::FramePairer pairer(bt::FramePairerConfig{18.0});
        const auto a1 = Frame(1, 1'000'000'000);
        const auto b1 = Frame(1, 1'008'000'000);
        BT_CHECK(pairer.Pair(a1, b1).valid);

        const auto a2 = Frame(2, 2'000'000'000);
        const auto b2 = Frame(2, 1'001'000'000);
        const auto mixed = pairer.PairRecent({a1, a2}, {b2});
        BT_CHECK(mixed.valid);
        BT_CHECK(mixed.degraded);
        BT_CHECK(mixed.reused_a);
        BT_CHECK(!mixed.reused_b);
        BT_CHECK(!mixed.skewed);
        BT_CHECK(mixed.camera_a->sequence == 1);
        BT_CHECK(mixed.camera_b->sequence == 2);
        BT_CHECK(mixed.reason == "reused_camera_a_frame_degraded");
    }

    return 0;
}
