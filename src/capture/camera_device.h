#pragma once

#include "capture/capture_health.h"
#include "capture/frame_slot.h"
#include "core/config.h"
#include "core/status.h"
#include "core/types.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace bt {

class NetworkCameraSource;

class CameraDevice {
public:
    CameraDevice(CameraId id, CameraConfig config);
    ~CameraDevice();

    Status Start();
    void Stop();

    std::shared_ptr<const FramePacket> GetLatestFrame() const;
    std::vector<std::shared_ptr<const FramePacket>> GetRecentFrames() const;
    CaptureHealthSnapshot GetHealthSnapshot() const;

private:
    void CaptureLoop();

    CameraId id_;
    CameraConfig config_;
    FrameSlot slot_;
    mutable std::mutex health_mutex_;
    CaptureHealthSnapshot health_{};
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
    mutable std::mutex network_source_mutex_;
    std::unique_ptr<NetworkCameraSource> network_source_;
};

} // namespace bt
