#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace bt::test {

#ifdef _WIN32
using FakeCameraSocket = SOCKET;
inline constexpr FakeCameraSocket kInvalidFakeCameraSocket = INVALID_SOCKET;
#else
using FakeCameraSocket = int;
inline constexpr FakeCameraSocket kInvalidFakeCameraSocket = -1;
#endif

inline bool EnsureFakeCameraSocketsReady() {
#ifdef _WIN32
    static const bool ready = [] {
        WSADATA data{};
        return WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }();
    return ready;
#else
    return true;
#endif
}

inline void CloseFakeCameraSocket(FakeCameraSocket socket) {
    if (socket == kInvalidFakeCameraSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

inline void AppendU32Be(std::vector<unsigned char>& out, std::uint32_t value) {
    out.push_back(static_cast<unsigned char>((value >> 24) & 0xff));
    out.push_back(static_cast<unsigned char>((value >> 16) & 0xff));
    out.push_back(static_cast<unsigned char>((value >> 8) & 0xff));
    out.push_back(static_cast<unsigned char>(value & 0xff));
}

inline void AppendU64Be(std::vector<unsigned char>& out, std::uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8) {
        out.push_back(static_cast<unsigned char>((value >> shift) & 0xff));
    }
}

inline std::vector<unsigned char> NetworkCameraMagic() {
    return {'B', 'T', 'M', 'J', 'P', 'E', 'G', '1', '\n'};
}

inline std::vector<unsigned char> NetworkCameraFrameHeader(std::uint32_t byte_count, std::uint64_t phone_ticks) {
    std::vector<unsigned char> out;
    out.reserve(12);
    AppendU32Be(out, byte_count);
    AppendU64Be(out, phone_ticks);
    return out;
}

inline int FindFreeLoopbackTcpPort() {
    if (!EnsureFakeCameraSocketsReady()) {
        return 0;
    }
    FakeCameraSocket socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket == kInvalidFakeCameraSocket) {
        return 0;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = 0;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        CloseFakeCameraSocket(socket);
        return 0;
    }
#ifdef _WIN32
    int len = sizeof(addr);
#else
    socklen_t len = sizeof(addr);
#endif
    if (::getsockname(socket, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        CloseFakeCameraSocket(socket);
        return 0;
    }
    const int port = ntohs(addr.sin_port);
    CloseFakeCameraSocket(socket);
    return port;
}

class FakeNetworkCameraClient {
public:
    FakeNetworkCameraClient() = default;
    ~FakeNetworkCameraClient() {
        Close();
    }

    FakeNetworkCameraClient(const FakeNetworkCameraClient&) = delete;
    FakeNetworkCameraClient& operator=(const FakeNetworkCameraClient&) = delete;

    bool ConnectLoopback(int port, std::chrono::milliseconds timeout = std::chrono::milliseconds(1500)) {
        if (!EnsureFakeCameraSocketsReady()) {
            return false;
        }
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            Close();
            socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (socket_ == kInvalidFakeCameraSocket) {
                return false;
            }
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(static_cast<std::uint16_t>(port));
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            if (::connect(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        Close();
        return false;
    }

    bool SendMagic() {
        if (SendBytes(NetworkCameraMagic())) {
            magic_sent_ = true;
            return true;
        }
        return false;
    }

    bool SendFrameHeader(std::uint32_t byte_count, std::uint64_t phone_ticks = 0) {
        return SendBytes(NetworkCameraFrameHeader(byte_count, phone_ticks));
    }

    bool SendFrame(const std::vector<unsigned char>& jpeg, std::uint64_t phone_ticks = 0) {
        return (magic_sent_ || SendMagic()) &&
            SendFrameHeader(static_cast<std::uint32_t>(jpeg.size()), phone_ticks) &&
            SendBytes(jpeg);
    }

    bool SendBytes(const std::vector<unsigned char>& bytes) {
        return SendBytes(bytes.data(), bytes.size());
    }

    bool SendBytes(const unsigned char* data, std::size_t size) {
        if (socket_ == kInvalidFakeCameraSocket) {
            return false;
        }
        std::size_t sent_total = 0;
        while (sent_total < size) {
            const int chunk = static_cast<int>(std::min<std::size_t>(size - sent_total, 64 * 1024));
            const int sent = ::send(socket_, reinterpret_cast<const char*>(data + sent_total), chunk, 0);
            if (sent <= 0) {
                return false;
            }
            sent_total += static_cast<std::size_t>(sent);
        }
        return true;
    }

    void Close() {
        CloseFakeCameraSocket(socket_);
        socket_ = kInvalidFakeCameraSocket;
        magic_sent_ = false;
    }

private:
    FakeCameraSocket socket_ = kInvalidFakeCameraSocket;
    bool magic_sent_ = false;
};

} // namespace bt::test
