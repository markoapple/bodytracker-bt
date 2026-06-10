#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "capture/camera_device.h"
#include "test_check.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

void CloseSocket(SocketHandle s) {
    if (s != kInvalidSocket) {
        closesocket(s);
    }
}

class SocketRuntime {
public:
    SocketRuntime() {
        WSADATA data{};
        ok_ = WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }
    ~SocketRuntime() {
        if (ok_) {
            WSACleanup();
        }
    }
    bool ok() const { return ok_; }

private:
    bool ok_ = false;
};
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

void CloseSocket(SocketHandle s) {
    if (s != kInvalidSocket) {
        close(s);
    }
}

class SocketRuntime {
public:
    bool ok() const { return true; }
};
#endif

class SocketOwner {
public:
    SocketOwner() = default;
    explicit SocketOwner(SocketHandle s) : socket_(s) {}
    ~SocketOwner() { reset(); }

    SocketOwner(const SocketOwner&) = delete;
    SocketOwner& operator=(const SocketOwner&) = delete;

    SocketOwner(SocketOwner&& other) noexcept : socket_(other.socket_) {
        other.socket_ = kInvalidSocket;
    }

    SocketOwner& operator=(SocketOwner&& other) noexcept {
        if (this != &other) {
            reset();
            socket_ = other.socket_;
            other.socket_ = kInvalidSocket;
        }
        return *this;
    }

    SocketHandle get() const { return socket_; }
    bool valid() const { return socket_ != kInvalidSocket; }

    void reset(SocketHandle replacement = kInvalidSocket) {
        if (socket_ != kInvalidSocket) {
            CloseSocket(socket_);
        }
        socket_ = replacement;
    }

private:
    SocketHandle socket_ = kInvalidSocket;
};

std::uint16_t ReserveLoopbackPort() {
    SocketOwner probe(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
    BT_CHECK(probe.valid());

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(0);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    BT_CHECK(bind(probe.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);

    sockaddr_in bound{};
#ifdef _WIN32
    int len = sizeof(bound);
#else
    socklen_t len = sizeof(bound);
#endif
    BT_CHECK(getsockname(probe.get(), reinterpret_cast<sockaddr*>(&bound), &len) == 0);
    return ntohs(bound.sin_port);
}

SocketOwner ConnectWithRetry(std::uint16_t port) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        SocketOwner client(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
        BT_CHECK(client.valid());

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (connect(client.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
            return client;
        }
        client.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return SocketOwner{};
}

void SendAll(SocketHandle socket, const void* data, std::size_t size) {
    const auto* p = static_cast<const char*>(data);
    std::size_t sent = 0;
    while (sent < size) {
        const int n = send(socket, p + sent, static_cast<int>(size - sent), 0);
        BT_CHECK(n > 0);
        sent += static_cast<std::size_t>(n);
    }
}

std::array<unsigned char, 12> FrameHeader(std::uint32_t bytes) {
    std::array<unsigned char, 12> header{};
    header[0] = static_cast<unsigned char>((bytes >> 24) & 0xff);
    header[1] = static_cast<unsigned char>((bytes >> 16) & 0xff);
    header[2] = static_cast<unsigned char>((bytes >> 8) & 0xff);
    header[3] = static_cast<unsigned char>(bytes & 0xff);
    return header;
}

std::vector<unsigned char> MakeJpeg() {
    cv::Mat image(16, 16, CV_8UC3, cv::Scalar(32, 96, 192));
    std::vector<unsigned char> jpeg;
    BT_CHECK(cv::imencode(".jpg", image, jpeg));
    BT_CHECK(!jpeg.empty());
    return jpeg;
}

} // namespace

int main() {
    SocketRuntime sockets;
    BT_CHECK(sockets.ok());

    const std::uint16_t port = ReserveLoopbackPort();
    bt::CameraConfig config;
    config.source = "network_mjpeg";
    config.network_bind_address = "127.0.0.1";
    config.network_port = static_cast<int>(port);
    config.network_read_timeout_ms = 60000;
    config.network_max_frame_bytes = 1024 * 1024;

    bt::CameraDevice camera(bt::CameraId::A, config);
    const auto start = camera.Start();
    BT_CHECK(start.ok());

    SocketOwner client = ConnectWithRetry(port);
    BT_CHECK(client.valid());

    static constexpr std::array<unsigned char, 9> kMagic{{'B', 'T', 'M', 'J', 'P', 'E', 'G', '1', '\n'}};
    SendAll(client.get(), kMagic.data(), kMagic.size());

    const auto jpeg = MakeJpeg();
    const auto header = FrameHeader(static_cast<std::uint32_t>(jpeg.size()));
    SendAll(client.get(), header.data(), header.size());
    SendAll(client.get(), jpeg.data(), jpeg.size());

    std::shared_ptr<const bt::FramePacket> delivered;
    for (int i = 0; i < 200; ++i) {
        delivered = camera.GetLatestFrame();
        if (delivered) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    BT_CHECK(delivered != nullptr);
    BT_CHECK(delivered->width == 16);
    BT_CHECK(delivered->height == 16);

    const auto partial_header = FrameHeader(128);
    SendAll(client.get(), partial_header.data(), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    const auto stop_start = std::chrono::steady_clock::now();
    camera.Stop();
    const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - stop_start).count();

    BT_CHECK(stop_ms < 350);

    const auto after_stop = camera.GetLatestFrame();
    BT_CHECK(after_stop != nullptr);
    BT_CHECK(after_stop->sequence == delivered->sequence);

    const auto health = camera.GetHealthSnapshot();
    BT_CHECK(!health.running);
    BT_CHECK(health.source_state == "stopped");
    BT_CHECK(health.last_degraded_reason == "source_stopping");
    BT_CHECK(health.last_frame_status == "stale_frame" || health.last_frame_status == "fresh_frame");
    return 0;
}
