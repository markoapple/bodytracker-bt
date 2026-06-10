#include "io/osc_sender.h"
#include "test_check.h"

#include <chrono>
#include <cstddef>
#include <string>

int main() {
    constexpr int kIterations = 10000;
    constexpr double kMaxElapsedMs = 1500.0;
    const std::string address = "/tracking/trackers/1/position";

    std::size_t total_bytes = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        const float value = static_cast<float>(i % 127) * 0.01f;
        const auto packet = bt::EncodeOscVector3MessageForTest(address, value, value + 1.0f, value + 2.0f);
        total_bytes += packet.size();
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(elapsed).count();

    BT_CHECK(total_bytes > 0);
    BT_CHECK(elapsed_ms < kMaxElapsedMs);
    return 0;
}
