#include "capture/network_camera_source.h"
#include "core/timing.h"
#include "fakes/fake_network_camera_client.h"
#include "test_check.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

bt::CameraConfig NetworkConfig(int port, int read_timeout_ms = 75) {
    bt::CameraConfig config;
    config.source = "network_mjpeg";
    config.network_bind_address = "127.0.0.1";
    config.network_port = port;
    config.network_read_timeout_ms = read_timeout_ms;
    config.network_max_frame_bytes = 1024 * 1024;
    return config;
}

std::vector<unsigned char> TinyJpeg() {
    cv::Mat image(4, 4, CV_8UC3, cv::Scalar(16, 32, 64));
    std::vector<unsigned char> jpeg;
    BT_CHECK(cv::imencode(".jpg", image, jpeg));
    BT_CHECK(!jpeg.empty());
    return jpeg;
}

class NetworkSourceHarness {
public:
    explicit NetworkSourceHarness(int port, int read_timeout_ms = 75)
        : source_(bt::CameraId::A, NetworkConfig(port, read_timeout_ms)) {}

    ~NetworkSourceHarness() {
        stop_.store(true);
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void Start() {
        worker_ = std::thread([this] {
            source_.Run(slot_, health_mutex_, health_, stop_);
            finished_.store(true);
        });
    }

    void RequestStop() {
        stop_.store(true);
    }

    bool WaitForFinished(std::chrono::milliseconds timeout) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (finished_.load()) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        return finished_.load();
    }

    void Join() {
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bt::CaptureHealthSnapshot Health() const {
        std::scoped_lock lock(health_mutex_);
        return health_;
    }

    std::shared_ptr<const bt::FramePacket> LatestFrame() const {
        return slot_.Load();
    }

    template <typename Predicate>
    bool WaitUntil(Predicate predicate, std::chrono::milliseconds timeout) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        return predicate();
    }

private:
    mutable std::mutex health_mutex_{};
    bt::CaptureHealthSnapshot health_{};
    bt::FrameSlot slot_{};
    std::atomic<bool> stop_{false};
    std::atomic<bool> finished_{false};
    bt::NetworkCameraSource source_;
    std::thread worker_{};
};

void StalledHeaderStopReturnsPromptly() {
    const int port = bt::test::FindFreeLoopbackTcpPort();
    BT_CHECK(port > 0);
    NetworkSourceHarness harness(port);
    harness.Start();
    BT_CHECK(harness.WaitUntil([&] { return harness.Health().opened; }, 1500ms));

    bt::test::FakeNetworkCameraClient client;
    BT_CHECK(client.ConnectLoopback(port));
    BT_CHECK(client.SendMagic());
    std::this_thread::sleep_for(150ms);

    harness.RequestStop();
    BT_CHECK(harness.WaitForFinished(2000ms));
    harness.Join();
    const auto health = harness.Health();
    BT_CHECK(health.read_failures >= 1);
    BT_CHECK(health.last_degraded_reason == "socket_timeout" ||
        health.last_degraded_reason == "source_stopping" ||
        health.last_degraded_reason == "socket_closed" ||
        health.last_degraded_reason == "partial_frame");
}

void PartialFramePreservesLastFiniteFrameThroughReconnect() {
    const int port = bt::test::FindFreeLoopbackTcpPort();
    BT_CHECK(port > 0);
    NetworkSourceHarness harness(port);
    harness.Start();
    BT_CHECK(harness.WaitUntil([&] { return harness.Health().opened; }, 1500ms));

    const auto jpeg = TinyJpeg();
    bt::test::FakeNetworkCameraClient first_client;
    BT_CHECK(first_client.ConnectLoopback(port));
    BT_CHECK(first_client.SendFrame(jpeg));
    BT_CHECK(harness.WaitUntil([&] { return static_cast<bool>(harness.LatestFrame()); }, 1500ms));
    first_client.Close();

    const auto first_frame = harness.LatestFrame();
    BT_CHECK(static_cast<bool>(first_frame));
    BT_CHECK(first_frame->sequence == 1);
    BT_CHECK(first_frame->width == 4);
    BT_CHECK(first_frame->height == 4);

    bt::test::FakeNetworkCameraClient partial_client;
    BT_CHECK(partial_client.ConnectLoopback(port));
    BT_CHECK(partial_client.SendMagic());
    BT_CHECK(partial_client.SendFrameHeader(static_cast<std::uint32_t>(jpeg.size() + 16)));
    BT_CHECK(partial_client.SendBytes(jpeg.data(), 2));
    std::this_thread::sleep_for(150ms);

    harness.RequestStop();
    BT_CHECK(harness.WaitForFinished(2000ms));
    harness.Join();

    const auto latest = harness.LatestFrame();
    BT_CHECK(static_cast<bool>(latest));
    BT_CHECK(latest->sequence == first_frame->sequence);
    BT_CHECK(latest->width == first_frame->width);
    BT_CHECK(latest->height == first_frame->height);
    const auto health = harness.Health();
    BT_CHECK(health.delivered_frames >= 1);
    BT_CHECK(health.read_failures >= 1);
    BT_CHECK(health.last_degraded_reason == "partial_frame" ||
        health.last_degraded_reason == "source_stopping");
    BT_CHECK(health.last_error_message.find("keeping last finite frame") != std::string::npos);
}


void MalformedHeaderIsDegradedNotFatal() {
    const int port = bt::test::FindFreeLoopbackTcpPort();
    BT_CHECK(port > 0);
    NetworkSourceHarness harness(port);
    harness.Start();
    BT_CHECK(harness.WaitUntil([&] { return harness.Health().opened; }, 1500ms));

    bt::test::FakeNetworkCameraClient client;
    BT_CHECK(client.ConnectLoopback(port));
    const std::vector<unsigned char> bad_magic{'B','A','D','M','J','P','E','G','\n'};
    BT_CHECK(client.SendBytes(bad_magic));
    BT_CHECK(harness.WaitUntil([&] { return harness.Health().read_failures >= 1; }, 1500ms));

    harness.RequestStop();
    BT_CHECK(harness.WaitForFinished(2000ms));
    harness.Join();
    const auto health = harness.Health();
    BT_CHECK(health.last_degraded_reason == "invalid_stream_header" ||
        health.last_degraded_reason == "source_stopping");
}

void ReconnectAcceptsSecondFiniteFrameAndReportsReconnect() {
    const int port = bt::test::FindFreeLoopbackTcpPort();
    BT_CHECK(port > 0);
    NetworkSourceHarness harness(port);
    harness.Start();
    BT_CHECK(harness.WaitUntil([&] { return harness.Health().opened; }, 1500ms));

    const auto jpeg = TinyJpeg();
    bt::test::FakeNetworkCameraClient first_client;
    BT_CHECK(first_client.ConnectLoopback(port));
    BT_CHECK(first_client.SendFrame(jpeg));
    BT_CHECK(harness.WaitUntil([&] {
        const auto frame = harness.LatestFrame();
        return frame && frame->sequence == 1;
    }, 1500ms));
    first_client.Close();

    bt::test::FakeNetworkCameraClient second_client;
    BT_CHECK(second_client.ConnectLoopback(port, 2500ms));
    BT_CHECK(second_client.SendFrame(jpeg));
    BT_CHECK(harness.WaitUntil([&] {
        const auto frame = harness.LatestFrame();
        return frame && frame->sequence >= 2;
    }, 2500ms));

    harness.RequestStop();
    BT_CHECK(harness.WaitForFinished(2000ms));
    harness.Join();
    const auto latest = harness.LatestFrame();
    BT_CHECK(static_cast<bool>(latest));
    BT_CHECK(latest->sequence >= 2);
    BT_CHECK(harness.Health().network_reconnect_count >= 1);
}

void PhoneTimestampDriftBecomesFrameAgeNotDiscard() {
    const int port = bt::test::FindFreeLoopbackTcpPort();
    BT_CHECK(port > 0);
    NetworkSourceHarness harness(port, 300);
    harness.Start();
    BT_CHECK(harness.WaitUntil([&] { return harness.Health().opened; }, 1500ms));

    const auto jpeg = TinyJpeg();
    const auto phone_t0 = static_cast<std::uint64_t>(std::max<std::int64_t>(1, bt::NowQpc().ticks - 300000000));
    bt::test::FakeNetworkCameraClient client;
    BT_CHECK(client.ConnectLoopback(port));
    BT_CHECK(client.SendFrame(jpeg, phone_t0));
    BT_CHECK(harness.WaitUntil([&] { return static_cast<bool>(harness.LatestFrame()); }, 1500ms));
    std::this_thread::sleep_for(120ms);
    BT_CHECK(client.SendFrame(jpeg, phone_t0 + 16666667ULL));
    BT_CHECK(harness.WaitUntil([&] {
        const auto frame = harness.LatestFrame();
        return frame && frame->sequence >= 2;
    }, 1500ms));

    const auto latest = harness.LatestFrame();
    BT_CHECK(static_cast<bool>(latest));
    const double age_ms = std::max(0.0, bt::QpcDeltaSeconds(latest->timestamp, bt::NowQpc()) * 1000.0);
    BT_CHECK(age_ms >= 40.0);

    harness.RequestStop();
    BT_CHECK(harness.WaitForFinished(2000ms));
    harness.Join();
    BT_CHECK(harness.Health().delivered_frames >= 2);
}

} // namespace

int main() {
    StalledHeaderStopReturnsPromptly();
    PartialFramePreservesLastFiniteFrameThroughReconnect();
    MalformedHeaderIsDegradedNotFatal();
    ReconnectAcceptsSecondFiniteFrameAndReportsReconnect();
    PhoneTimestampDriftBecomesFrameAgeNotDiscard();
    return 0;
}
