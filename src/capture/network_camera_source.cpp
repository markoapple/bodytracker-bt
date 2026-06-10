#include "capture/network_camera_source.h"
#include "core/logging.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace bt {
namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

void CloseSocket(SocketHandle s) {
    if (s != kInvalidSocket) {
        closesocket(s);
    }
}

void ShutdownSocket(SocketHandle s) {
    if (s != kInvalidSocket) {
        shutdown(s, SD_BOTH);
    }
}

std::uintptr_t PortableSocketHandle(SocketHandle s) {
    return static_cast<std::uintptr_t>(s);
}

SocketHandle SocketFromPortableHandle(std::uintptr_t s) {
    return static_cast<SocketHandle>(s);
}

std::string LastSocketError() {
    return "winsock_error_" + std::to_string(WSAGetLastError());
}

bool WouldBlockOrTimedOut() {
    const int e = WSAGetLastError();
    return e == WSAEWOULDBLOCK || e == WSAETIMEDOUT;
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

void ShutdownSocket(SocketHandle s) {
    if (s != kInvalidSocket) {
        shutdown(s, SHUT_RDWR);
    }
}

std::uintptr_t PortableSocketHandle(SocketHandle s) {
    return static_cast<std::uintptr_t>(s);
}

SocketHandle SocketFromPortableHandle(std::uintptr_t s) {
    return static_cast<SocketHandle>(s);
}

std::string LastSocketError() {
    return std::strerror(errno);
}

bool WouldBlockOrTimedOut() {
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
}

class SocketRuntime {
public:
    bool ok() const { return true; }
};
#endif

class SocketOwner {
public:
    SocketOwner() = default;
    explicit SocketOwner(SocketHandle s) : s_(s) {}
    ~SocketOwner() { reset(); }

    SocketOwner(const SocketOwner&) = delete;
    SocketOwner& operator=(const SocketOwner&) = delete;

    SocketOwner(SocketOwner&& other) noexcept : s_(other.s_) {
        other.s_ = kInvalidSocket;
    }
    SocketOwner& operator=(SocketOwner&& other) noexcept {
        if (this != &other) {
            reset();
            s_ = other.s_;
            other.s_ = kInvalidSocket;
        }
        return *this;
    }

    SocketHandle get() const { return s_; }
    bool valid() const { return s_ != kInvalidSocket; }
    void reset(SocketHandle replacement = kInvalidSocket) {
        if (s_ != kInvalidSocket) {
            CloseSocket(s_);
        }
        s_ = replacement;
    }

private:
    SocketHandle s_ = kInvalidSocket;
};

void UpdateHealth(
    std::mutex& mutex,
    CaptureHealthSnapshot& health,
    const std::function<void(CaptureHealthSnapshot&)>& fn) {
    std::scoped_lock lock(mutex);
    fn(health);
}

bool SetSocketTimeout(SocketHandle s, int timeout_ms) {
    const int clamped_ms = std::max(50, timeout_ms);
#ifdef _WIN32
    const DWORD tv = static_cast<DWORD>(clamped_ms);
    return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv)) == 0 &&
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv)) == 0;
#else
    timeval tv{};
    tv.tv_sec = clamped_ms / 1000;
    tv.tv_usec = (clamped_ms % 1000) * 1000;
    return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0 &&
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) == 0;
#endif
}

constexpr int kNetworkShutdownPollMs = 25;

enum class SocketWaitStatus {
    Ready,
    TimedOut,
    Stopped,
    Error
};

enum class ReadExactStatus {
    Complete,
    TimedOut,
    Stopped,
    PeerClosed,
    SocketError
};

struct ReadExactResult {
    ReadExactStatus status = ReadExactStatus::Complete;
    std::size_t bytes_read = 0;
    std::string error;

    bool complete() const { return status == ReadExactStatus::Complete; }
};

SocketWaitStatus WaitReadable(
    SocketHandle s,
    const std::atomic<bool>& stop_requested,
    int wait_ms,
    std::string& error) {
    if (stop_requested) {
        return SocketWaitStatus::Stopped;
    }

    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(s, &read_set);
    timeval tv{};
    tv.tv_sec = wait_ms / 1000;
    tv.tv_usec = (wait_ms % 1000) * 1000;
    const int ready = select(static_cast<int>(s + 1), &read_set, nullptr, nullptr, &tv);
    if (stop_requested) {
        return SocketWaitStatus::Stopped;
    }
    if (ready > 0 && FD_ISSET(s, &read_set)) {
        return SocketWaitStatus::Ready;
    }
    if (ready == 0) {
        return SocketWaitStatus::TimedOut;
    }
    if (WouldBlockOrTimedOut()) {
        return SocketWaitStatus::TimedOut;
    }
    error = LastSocketError();
    return SocketWaitStatus::Error;
}

const char* ReadFailureReason(const ReadExactResult& read) {
    if (read.status == ReadExactStatus::Stopped) {
        return "source_stopping";
    }
    if (read.bytes_read > 0 && read.status != ReadExactStatus::Complete) {
        return "partial_frame";
    }
    if (read.status == ReadExactStatus::TimedOut) {
        return "socket_timeout";
    }
    if (read.status == ReadExactStatus::PeerClosed) {
        return "socket_closed";
    }
    if (read.status == ReadExactStatus::SocketError) {
        return "socket_error";
    }
    return "";
}

SocketOwner CreateListener(const CameraConfig& config, std::string& error) {
    SocketOwner listener(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
    if (!listener.valid()) {
        error = "socket_create_failed:" + LastSocketError();
        return listener;
    }

    int yes = 1;
    setsockopt(listener.get(), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(config.network_port));
    if (config.network_bind_address.empty() || config.network_bind_address == "0.0.0.0") {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        if (inet_pton(AF_INET, config.network_bind_address.c_str(), &addr.sin_addr) != 1) {
            error = "network_bind_address must be an IPv4 address";
            listener.reset();
            return listener;
        }
    }

    if (bind(listener.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        error = "bind_failed:" + LastSocketError();
        listener.reset();
        return listener;
    }
    if (listen(listener.get(), 1) != 0) {
        error = "listen_failed:" + LastSocketError();
        listener.reset();
        return listener;
    }
    return listener;
}

SocketOwner AcceptWithPoll(SocketHandle listener, const std::atomic<bool>& stop_requested) {
    while (!stop_requested) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(listener, &read_set);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000;
        const int ready = select(static_cast<int>(listener + 1), &read_set, nullptr, nullptr, &tv);
        if (ready > 0 && FD_ISSET(listener, &read_set)) {
            sockaddr_in remote{};
#ifdef _WIN32
            int len = sizeof(remote);
#else
            socklen_t len = sizeof(remote);
#endif
            SocketOwner client(accept(listener, reinterpret_cast<sockaddr*>(&remote), &len));
            return client;
        }
    }
    return SocketOwner{};
}

ReadExactResult ReadExact(
    SocketHandle s,
    void* dst,
    std::size_t bytes,
    const std::atomic<bool>& stop_requested,
    int timeout_ms) {
    auto* p = static_cast<unsigned char*>(dst);
    std::size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(1, timeout_ms));
    while (offset < bytes) {
        if (stop_requested) {
            return {ReadExactStatus::Stopped, offset, {}};
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return {ReadExactStatus::TimedOut, offset, {}};
        }
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        std::string wait_error;
        const auto wait = WaitReadable(s, stop_requested, std::min(kNetworkShutdownPollMs, std::max(1, remaining_ms)), wait_error);
        if (wait == SocketWaitStatus::Stopped) {
            return {ReadExactStatus::Stopped, offset, {}};
        }
        if (wait == SocketWaitStatus::TimedOut) {
            continue;
        }
        if (wait == SocketWaitStatus::Error) {
            return {ReadExactStatus::SocketError, offset, wait_error};
        }

        const int n = recv(s, reinterpret_cast<char*>(p + offset), static_cast<int>(bytes - offset), 0);
        if (n > 0) {
            offset += static_cast<std::size_t>(n);
            continue;
        }
        if (n == 0) {
            return {ReadExactStatus::PeerClosed, offset, {}};
        }
        if (WouldBlockOrTimedOut()) {
            continue;
        }
        return {ReadExactStatus::SocketError, offset, LastSocketError()};
    }
    return {ReadExactStatus::Complete, offset, {}};
}

std::uint32_t ReadU32Be(const unsigned char* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) |
        (static_cast<std::uint32_t>(p[1]) << 16) |
        (static_cast<std::uint32_t>(p[2]) << 8) |
        static_cast<std::uint32_t>(p[3]);
}

std::uint64_t ReadU64Be(const unsigned char* p) {
    return (static_cast<std::uint64_t>(p[0]) << 56) |
        (static_cast<std::uint64_t>(p[1]) << 48) |
        (static_cast<std::uint64_t>(p[2]) << 40) |
        (static_cast<std::uint64_t>(p[3]) << 32) |
        (static_cast<std::uint64_t>(p[4]) << 24) |
        (static_cast<std::uint64_t>(p[5]) << 16) |
        (static_cast<std::uint64_t>(p[6]) << 8) |
        static_cast<std::uint64_t>(p[7]);
}

void SleepBriefly(const std::atomic<bool>& stop_requested, int total_ms) {
    const int step_ms = 50;
    int elapsed = 0;
    while (!stop_requested && elapsed < total_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
        elapsed += step_ms;
    }
}

} // namespace

NetworkCameraSource::NetworkCameraSource(CameraId id, CameraConfig config)
    : id_(id), config_(std::move(config)) {
}

void NetworkCameraSource::RequestStop() {
    std::scoped_lock lock(active_socket_mutex_);
    const auto invalid = std::numeric_limits<std::uintptr_t>::max();
    if (active_client_socket_ != invalid) {
        ShutdownSocket(SocketFromPortableHandle(active_client_socket_));
    }
    if (active_listener_socket_ != invalid) {
        ShutdownSocket(SocketFromPortableHandle(active_listener_socket_));
    }
}

void NetworkCameraSource::SetActiveListener(std::uintptr_t socket_handle) {
    std::scoped_lock lock(active_socket_mutex_);
    active_listener_socket_ = socket_handle;
}

void NetworkCameraSource::ClearActiveListener(std::uintptr_t socket_handle) {
    std::scoped_lock lock(active_socket_mutex_);
    if (active_listener_socket_ == socket_handle) {
        active_listener_socket_ = std::numeric_limits<std::uintptr_t>::max();
    }
}

void NetworkCameraSource::SetActiveClient(std::uintptr_t socket_handle) {
    std::scoped_lock lock(active_socket_mutex_);
    active_client_socket_ = socket_handle;
}

void NetworkCameraSource::ClearActiveClient(std::uintptr_t socket_handle) {
    std::scoped_lock lock(active_socket_mutex_);
    if (active_client_socket_ == socket_handle) {
        active_client_socket_ = std::numeric_limits<std::uintptr_t>::max();
    }
}

void NetworkCameraSource::Run(
    FrameSlot& slot,
    std::mutex& health_mutex,
    CaptureHealthSnapshot& health,
    const std::atomic<bool>& stop_requested) {

    (void)id_;
    SocketRuntime sockets;
    if (!sockets.ok()) {
        UpdateHealth(health_mutex, health, [](CaptureHealthSnapshot& h) {
            h.running = true;
            h.opened = false;
            h.open_failures += 1;
            h.backend_name = "network_mjpeg_tcp";
            h.source_state = "degraded";
            h.last_degraded_reason = "open_failed";
            h.last_error_message = "WSAStartup failed; network camera will retry only after restart";
        });
        while (!stop_requested) {
            SleepBriefly(stop_requested, 1000);
        }
        return;
    }

    std::uint64_t sequence = 0;
    std::uint64_t accepted_sessions = 0;
    QpcTimestamp previous_frame_time{};
    double previous_frame_interval_ms = 0.0;
    bool phone_clock_offset_ready = false;
    std::int64_t phone_to_pc_offset_ticks = std::numeric_limits<std::int64_t>::max();

    while (!stop_requested) {
        std::string listen_error;
        SocketOwner listener = CreateListener(config_, listen_error);
        if (!listener.valid()) {
            UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                h.running = true;
                h.opened = false;
                h.open_failures += 1;
                h.backend_name = "network_mjpeg_tcp";
                h.source_state = "degraded";
                h.last_degraded_reason = "open_failed";
                h.last_error_message = listen_error;
            });
            SleepBriefly(stop_requested, 750);
            continue;
        }

        SetActiveListener(PortableSocketHandle(listener.get()));

        UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
            h.running = true;
            h.opened = true;
            h.backend_api = 0;
            h.backend_name = "network_mjpeg_tcp:" + config_.network_bind_address + ":" + std::to_string(config_.network_port);
            h.source_state = "waiting";
            if (h.last_degraded_reason.empty() && h.last_error_message.empty()) {
                h.last_error_message = "waiting_for_phone_stream";
            }
        });

        SocketOwner client = AcceptWithPoll(listener.get(), stop_requested);
        ClearActiveListener(PortableSocketHandle(listener.get()));
        if (!client.valid()) {
            continue;
        }
        listener.reset();
        SetActiveClient(PortableSocketHandle(client.get()));
        ++accepted_sessions;
        const bool timeout_ready = SetSocketTimeout(client.get(), config_.network_read_timeout_ms);
        UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
            h.running = true;
            h.opened = true;
            h.source_state = timeout_ready ? "connected" : "degraded";
            h.network_accept_count = accepted_sessions;
            h.network_reconnect_count = accepted_sessions > 0 ? accepted_sessions - 1 : 0;
            h.last_degraded_reason = timeout_ready ? "" : "socket_timeout_setup_failed";
            h.last_error_message = timeout_ready
                ? "phone_stream_connected"
                : "phone_stream_connected; socket timeout setup failed; using stop-aware poll fallback";
            RefreshNetworkCaptureHealthMessage(h);
        });

        static constexpr std::array<unsigned char, 9> kMagic{{'B','T','M','J','P','E','G','1','\n'}};
        std::array<unsigned char, kMagic.size()> magic{};
        const auto magic_read = ReadExact(client.get(), magic.data(), magic.size(), stop_requested, config_.network_read_timeout_ms);
        if (!magic_read.complete() || magic != kMagic) {
            UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                h.read_failures += 1;
                h.consecutive_read_failures += 1;
                h.network_stream_header_failures += 1;
                h.source_state = magic_read.status == ReadExactStatus::Stopped ? "stopping" : "degraded";
                h.last_degraded_reason = magic_read.complete() ? "invalid_stream_header" : ReadFailureReason(magic_read);
                h.last_error_message = h.last_degraded_reason == "source_stopping"
                    ? "source_stopping"
                    : "network camera stream header failed: " + h.last_degraded_reason;
                RefreshNetworkCaptureHealthMessage(h);
            });
            ClearActiveClient(PortableSocketHandle(client.get()));
            continue;
        }

        UpdateHealth(health_mutex, health, [](CaptureHealthSnapshot& h) {
            h.network_stream_header_count += 1;
            h.source_state = "receiving";
            h.last_degraded_reason.clear();
            h.last_error_message.clear();
            h.consecutive_read_failures = 0;
            RefreshNetworkCaptureHealthMessage(h);
        });

        while (!stop_requested) {
            std::array<unsigned char, 12> header{};
            const auto read_start = NowQpc();
            const auto header_read = ReadExact(client.get(), header.data(), header.size(), stop_requested, config_.network_read_timeout_ms);
            if (!header_read.complete()) {
                const double read_ms = QpcDeltaSeconds(read_start, NowQpc()) * 1000.0;
                UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                    h.running = true;
                    h.opened = true;
                    h.last_read_ms = read_ms;
                    h.read_failures += 1;
                    h.consecutive_read_failures += 1;
                    h.network_frame_header_failures += 1;
                    h.source_state = header_read.status == ReadExactStatus::Stopped ? "stopping" : "degraded";
                    h.last_degraded_reason = ReadFailureReason(header_read);
                    h.last_error_message = h.last_degraded_reason + std::string(": network camera frame header unavailable; keeping last finite frame available");
                    RefreshNetworkCaptureHealthMessage(h);
                });
                break;
            }
            UpdateHealth(health_mutex, health, [](CaptureHealthSnapshot& h) {
                h.network_frame_header_count += 1;
                RefreshNetworkCaptureHealthMessage(h);
            });

            const std::uint32_t frame_bytes = ReadU32Be(header.data());
            const std::uint64_t phone_capture_ticks = ReadU64Be(header.data() + 4);
            if (frame_bytes == 0 || frame_bytes > static_cast<std::uint32_t>(config_.network_max_frame_bytes)) {
                UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                    h.read_failures += 1;
                    h.consecutive_read_failures += 1;
                    h.network_bad_frame_size_count += 1;
                    h.source_state = "degraded";
                    h.last_degraded_reason = "bad_frame_size";
                    h.last_error_message = "network camera frame length outside configured bounds: " +
                        std::to_string(frame_bytes) + " > " + std::to_string(config_.network_max_frame_bytes);
                    RefreshNetworkCaptureHealthMessage(h);
                });
                break;
            }

            std::vector<unsigned char> jpeg(frame_bytes);
            const auto payload_read = ReadExact(client.get(), jpeg.data(), jpeg.size(), stop_requested, config_.network_read_timeout_ms);
            if (!payload_read.complete()) {
                const double read_ms = QpcDeltaSeconds(read_start, NowQpc()) * 1000.0;
                UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                    h.running = true;
                    h.opened = true;
                    h.last_read_ms = read_ms;
                    h.read_failures += 1;
                    h.consecutive_read_failures += 1;
                    h.network_frame_payload_failures += 1;
                    h.source_state = payload_read.status == ReadExactStatus::Stopped ? "stopping" : "degraded";
                    h.last_degraded_reason = ReadFailureReason(payload_read);
                    h.last_error_message = h.last_degraded_reason + std::string(": network camera JPEG payload unavailable; keeping last finite frame available");
                    RefreshNetworkCaptureHealthMessage(h);
                });
                break;
            }
            UpdateHealth(health_mutex, health, [](CaptureHealthSnapshot& h) {
                h.network_frame_payload_count += 1;
                RefreshNetworkCaptureHealthMessage(h);
            });

            const auto receive_done = NowQpc();
            const double read_ms = QpcDeltaSeconds(read_start, receive_done) * 1000.0;
            cv::Mat frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
            if (frame.empty()) {
                UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                    h.running = true;
                    h.opened = true;
                    h.last_read_ms = read_ms;
                    h.read_failures += 1;
                    h.consecutive_read_failures += 1;
                    h.network_decode_failures += 1;
                    h.source_state = "degraded";
                    h.last_degraded_reason = "decode_failed";
                    h.last_decode_error = "network camera JPEG decode produced no pixels";
                    h.last_error_message = h.last_decode_error;
                    RefreshNetworkCaptureHealthMessage(h);
                });
                continue;
            }

            auto packet = std::make_shared<FramePacket>();
            packet->bgr = std::move(frame);
            packet->width = packet->bgr.cols;
            packet->height = packet->bgr.rows;
            packet->sequence = ++sequence;
            if (phone_capture_ticks > 0 && phone_capture_ticks <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
                const auto phone_ticks = static_cast<std::int64_t>(phone_capture_ticks);
                const std::int64_t observed_offset = receive_done.ticks - phone_ticks;
                if (!phone_clock_offset_ready || observed_offset < phone_to_pc_offset_ticks) {
                    // Keep the lowest observed receive-phone delta as the least-bad LAN latency baseline.
                    // Later spikes make packet->timestamp older, so the existing age confidence scale
                    // weakens the pixels instead of throwing them away.
                    phone_to_pc_offset_ticks = observed_offset;
                    phone_clock_offset_ready = true;
                }
                packet->timestamp = QpcTimestamp{phone_ticks + phone_to_pc_offset_ticks};
                if (packet->timestamp.ticks > receive_done.ticks) {
                    packet->timestamp = receive_done;
                }
            } else {
                packet->timestamp = read_start;
            }
            slot.Store(packet);
            if (packet->sequence <= 3 || packet->sequence % 300 == 0) {
                Logger::Instance().Write(
                    LogLevel::Info,
                    std::string("network camera ") + (id_ == CameraId::A ? "A" : "B") +
                        " receiving frame #" + std::to_string(packet->sequence) +
                        " " + std::to_string(packet->width) + "x" + std::to_string(packet->height) +
                        " jpeg_bytes=" + std::to_string(jpeg.size()) +
                        " read_ms=" + std::to_string(read_ms));
            }

            UpdateHealth(health_mutex, health, [&](CaptureHealthSnapshot& h) {
                h.running = true;
                h.opened = true;
                h.actual_width = packet->width;
                h.actual_height = packet->height;
                if (previous_frame_time.ticks != 0) {
                    const double dt = QpcDeltaSeconds(previous_frame_time, packet->timestamp);
                    if (dt > 0.0001) {
                        const double instant_fps = 1.0 / dt;
                        const double interval_ms = dt * 1000.0;
                        if (previous_frame_interval_ms > 0.0) {
                            const double jitter_sample = std::abs(interval_ms - previous_frame_interval_ms);
                            h.network_jitter_ms = h.network_jitter_ms > 0.0
                                ? (0.85 * h.network_jitter_ms + 0.15 * jitter_sample)
                                : jitter_sample;
                        }
                        previous_frame_interval_ms = interval_ms;
                        h.actual_fps = h.actual_fps > 0.0 ? (0.90 * h.actual_fps + 0.10 * instant_fps) : instant_fps;
                    }
                }
                previous_frame_time = packet->timestamp;
                h.last_read_ms = read_ms;
                h.delivered_frames += 1;
                h.network_decode_successes += 1;
                h.consecutive_read_failures = 0;
                h.last_frame_timestamp = packet->timestamp;
                h.last_frame_age_ms = std::max(0.0, QpcDeltaSeconds(packet->timestamp, NowQpc()) * 1000.0);
                h.source_state = "receiving";
                h.last_frame_status = "fresh_frame";
                h.last_degraded_reason.clear();
                h.last_decode_error.clear();
                h.last_error_message.clear();
                RefreshNetworkCaptureHealthMessage(h);
            });
        }
        ClearActiveClient(PortableSocketHandle(client.get()));
    }
}

} // namespace bt
