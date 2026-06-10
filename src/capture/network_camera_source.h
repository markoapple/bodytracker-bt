#pragma once

#include "capture/capture_health.h"
#include "capture/frame_slot.h"
#include "core/config.h"
#include "core/types.h"

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>

namespace bt {

class NetworkCameraSource {
public:
    NetworkCameraSource(CameraId id, CameraConfig config);

    void RequestStop();

    void Run(
        FrameSlot& slot,
        std::mutex& health_mutex,
        CaptureHealthSnapshot& health,
        const std::atomic<bool>& stop_requested);

private:
    void SetActiveListener(std::uintptr_t socket_handle);
    void ClearActiveListener(std::uintptr_t socket_handle);
    void SetActiveClient(std::uintptr_t socket_handle);
    void ClearActiveClient(std::uintptr_t socket_handle);

    CameraId id_;
    CameraConfig config_;
    mutable std::mutex active_socket_mutex_;
    std::uintptr_t active_listener_socket_ = std::numeric_limits<std::uintptr_t>::max();
    std::uintptr_t active_client_socket_ = std::numeric_limits<std::uintptr_t>::max();
};

} // namespace bt
