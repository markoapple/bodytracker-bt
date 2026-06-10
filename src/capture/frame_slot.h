#pragma once

#include "capture/frame.h"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <utility>

namespace bt {

class FrameSlot {
public:
    void Store(std::shared_ptr<const FramePacket> frame) {
        std::scoped_lock lock(mutex_);
        latest_ = std::move(frame);
        if (latest_) {
            history_.push_back(latest_);
            while (history_.size() > kMaxHistory) {
                history_.pop_front();
            }
        }
        replacements_ += 1;
    }

    std::shared_ptr<const FramePacket> Load() const {
        std::scoped_lock lock(mutex_);
        return latest_;
    }

    std::vector<std::shared_ptr<const FramePacket>> LoadRecent(std::size_t max_count = kMaxHistory) const {
        std::scoped_lock lock(mutex_);
        std::vector<std::shared_ptr<const FramePacket>> out;
        const std::size_t n = std::min(max_count, history_.size());
        out.reserve(n);
        const auto begin = history_.end() - static_cast<std::ptrdiff_t>(n);
        for (auto it = begin; it != history_.end(); ++it) {
            out.push_back(*it);
        }
        return out;
    }

    std::uint64_t replacements() const {
        std::scoped_lock lock(mutex_);
        return replacements_;
    }

private:
    static constexpr std::size_t kMaxHistory = 8;
    mutable std::mutex mutex_;
    std::shared_ptr<const FramePacket> latest_;
    std::deque<std::shared_ptr<const FramePacket>> history_;
    std::uint64_t replacements_ = 0;
};

} // namespace bt
