#include "capture/camera_device.h"
#include "capture/network_camera_source.h"

#include <opencv2/videoio.hpp>

#include <chrono>
#include <algorithm>
#include <thread>
#include <utility>
#include <vector>

namespace bt {
namespace {

struct CaptureBackendCandidate {
    int api = cv::CAP_ANY;
    const char* name = "any";
};

std::vector<CaptureBackendCandidate> CaptureBackends() {
#ifdef _WIN32
    return {
        {cv::CAP_DSHOW, "dshow"},
        {cv::CAP_MSMF, "msmf"},
        {cv::CAP_ANY, "any"}
    };
#else
    return {{cv::CAP_ANY, "any"}};
#endif
}

} // namespace

CameraDevice::CameraDevice(CameraId id, CameraConfig config)
    : id_(id), config_(std::move(config)) {
}

CameraDevice::~CameraDevice() {
    Stop();
}

Status CameraDevice::Start() {
    if (worker_.joinable()) {
        return Status::Error(StatusCode::FailedPrecondition, "Camera thread already running");
    }
    stop_requested_ = false;
    worker_ = std::thread([this]() { CaptureLoop(); });

    return Status::OK();
}

void CameraDevice::Stop() {
    stop_requested_ = true;
    {
        std::scoped_lock lock(health_mutex_);
        health_.source_state = "stopping";
        health_.last_degraded_reason = "source_stopping";
        health_.last_error_message = "source_stopping";
    }
    {
        std::scoped_lock lock(network_source_mutex_);
        if (network_source_) {
            network_source_->RequestStop();
        }
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    std::scoped_lock lock(health_mutex_);
    health_.running = false;
    health_.opened = false;
    health_.source_state = "stopped";
}

std::shared_ptr<const FramePacket> CameraDevice::GetLatestFrame() const {
    return slot_.Load();
}

std::vector<std::shared_ptr<const FramePacket>> CameraDevice::GetRecentFrames() const {
    return slot_.LoadRecent();
}

CaptureHealthSnapshot CameraDevice::GetHealthSnapshot() const {
    std::scoped_lock lock(health_mutex_);
    auto out = health_;
    out.slot_replacements = slot_.replacements();
    if (out.last_frame_timestamp.ticks != 0) {
        const double age_ms = QpcDeltaSeconds(out.last_frame_timestamp, NowQpc()) * 1000.0;
        out.last_frame_age_ms = age_ms;
        if (config_.source == "network_mjpeg" && age_ms > static_cast<double>(config_.network_read_timeout_ms)) {
            out.last_frame_status = "stale_frame";
            if (out.last_degraded_reason.empty()) {
                out.last_degraded_reason = "stale_frame";
            }
            if (out.source_state == "receiving") {
                out.source_state = "degraded";
            }
        } else {
            out.last_frame_status = "fresh_frame";
        }
    }
    return out;
}

void CameraDevice::CaptureLoop() {
    if (config_.source == "network_mjpeg") {
        auto source = std::make_unique<NetworkCameraSource>(id_, config_);
        NetworkCameraSource* active_source = source.get();
        {
            std::scoped_lock lock(network_source_mutex_);
            network_source_ = std::move(source);
        }
        active_source->Run(slot_, health_mutex_, health_, stop_requested_);
        {
            std::scoped_lock lock(network_source_mutex_);
            network_source_.reset();
        }
        return;
    }

    cv::VideoCapture cap;
    const auto open_capture = [&]() -> bool {
        cap.release();
        CaptureBackendCandidate used_backend{};
        for (const auto& backend : CaptureBackends()) {
            cap.open(config_.device_index, backend.api);
            if (cap.isOpened()) {
                used_backend = backend;
                break;
            }
            cap.release();
        }
        {
            std::scoped_lock lock(health_mutex_);
            health_.opened = cap.isOpened();
            health_.running = cap.isOpened();
            if (!cap.isOpened()) {
                health_.open_failures += 1;
                health_.source_state = "degraded";
                health_.last_degraded_reason = "open_failed";
                health_.last_error_message = "OpenCV could not open camera with DSHOW, MSMF, or CAP_ANY";
                return false;
            }
            health_.backend_api = used_backend.api;
            health_.backend_name = used_backend.name;
            health_.source_state = "receiving";
            health_.last_degraded_reason.clear();
            health_.last_error_message.clear();
        }

        cap.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
        cap.set(cv::CAP_PROP_FPS, config_.fps);
        return true;
    };

    if (!open_capture()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    std::uint64_t sequence = 0;
    while (!stop_requested_) {
        cv::Mat frame;
        const auto read_start = NowQpc();
        if (!cap.isOpened()) {
            open_capture();
        }
        const bool read_ok = cap.isOpened() && cap.read(frame);
        const double read_ms = QpcDeltaSeconds(read_start, NowQpc()) * 1000.0;
        if (!read_ok || frame.empty()) {
            std::uint64_t consecutive = 0;
            {
                std::scoped_lock lock(health_mutex_);
                health_.running = true;
                health_.last_read_ms = read_ms;
                health_.read_failures += 1;
                health_.consecutive_read_failures += 1;
                consecutive = health_.consecutive_read_failures;
                health_.source_state = "degraded";
                health_.last_degraded_reason = "read_failed";
                health_.last_error_message = "Camera read failed";
            }

            if (consecutive >= 30) {
                cap.release();
                {
                    std::scoped_lock lock(health_mutex_);
                    health_.opened = false;
                    health_.running = true;
                    health_.source_state = "degraded";
                    health_.last_degraded_reason = "read_failed";
                    health_.last_error_message = "Camera read failed repeatedly; reopening";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                if (!stop_requested_) {
                    open_capture();
                }
                continue;
            }

            const auto backoff_ms = std::min<std::uint64_t>(100, 2 + consecutive * 2);
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            continue;
        }

        auto packet = std::make_shared<FramePacket>();
        packet->bgr = frame;
        packet->width = frame.cols;
        packet->height = frame.rows;
        packet->sequence = ++sequence;
        packet->timestamp = NowQpc();
        slot_.Store(packet);

        std::scoped_lock lock(health_mutex_);
        health_.opened = true;
        health_.running = true;
        health_.actual_width = frame.cols;
        health_.actual_height = frame.rows;
        health_.actual_fps = cap.get(cv::CAP_PROP_FPS);
        health_.last_read_ms = read_ms;
        health_.delivered_frames += 1;
        health_.consecutive_read_failures = 0;
        health_.last_frame_timestamp = packet->timestamp;
        health_.source_state = "receiving";
        health_.last_frame_status = "fresh_frame";
        health_.last_degraded_reason.clear();
        health_.last_error_message.clear();
    }
}

} // namespace bt
