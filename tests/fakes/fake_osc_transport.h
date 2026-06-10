#pragma once

#include "core/status.h"

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bt::test {

struct OscMessageAttempt {
    std::string address;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

class FakeOscTransportHook {
public:
    void FailAddress(std::string address, std::string message = "fake OSC transport failure") {
        failures_[std::move(address)] = Status::Error(StatusCode::DeviceUnavailable, std::move(message));
    }

    [[nodiscard]] std::function<Status(const std::string&, float, float, float)> Hook() {
        return [this](const std::string& address, float x, float y, float z) {
            return Send(address, x, y, z);
        };
    }

    [[nodiscard]] const std::vector<OscMessageAttempt>& Attempts() const noexcept {
        return attempts_;
    }

private:
    Status Send(const std::string& address, float x, float y, float z) {
        attempts_.push_back(OscMessageAttempt{address, x, y, z});
        const auto it = failures_.find(address);
        if (it != failures_.end()) {
            return it->second;
        }
        return Status::OK();
    }

    std::unordered_map<std::string, Status> failures_{};
    std::vector<OscMessageAttempt> attempts_{};
};

} // namespace bt::test
