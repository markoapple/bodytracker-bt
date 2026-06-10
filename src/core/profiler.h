#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>

namespace bt {

struct ProfilerFrameStats {
    double total_ms = 0.0;
    double capture_ms = 0.0;
    double frame_pair_ms = 0.0;
    double preprocess_ms = 0.0;
    double inference_ms = 0.0;
    double onnx_ms = 0.0;
    double decode_ms = 0.0;
    double pipeline_ms = 0.0;
    double solver_ms = 0.0;
    double osc_ms = 0.0;
    double ui_publish_ms = 0.0;
};

struct ProfilerStageStats {
    double last_ms = 0.0;
    double avg_ms = 0.0;
    double p95_ms = 0.0;
    double max_ms = 0.0;
    double budget_ms = 0.0;
    bool over_budget = false;
};

struct ProfilerBudget {
    double total_ms = 16.67;
    double capture_ms = 4.0;
    double frame_pair_ms = 1.0;
    double preprocess_ms = 2.0;
    double inference_ms = 8.0;
    double onnx_ms = 7.0;
    double decode_ms = 1.0;
    double pipeline_ms = 6.0;
    double solver_ms = 4.0;
    double osc_ms = 0.5;
    double ui_publish_ms = 1.0;
};

struct ProfilerSnapshot {
    std::size_t sample_count = 0;
    ProfilerStageStats total{};
    ProfilerStageStats capture{};
    ProfilerStageStats frame_pair{};
    ProfilerStageStats preprocess{};
    ProfilerStageStats inference{};
    ProfilerStageStats onnx{};
    ProfilerStageStats decode{};
    ProfilerStageStats pipeline{};
    ProfilerStageStats solver{};
    ProfilerStageStats osc{};
    ProfilerStageStats ui_publish{};
    std::string bottleneck_stage = "none";
    double bottleneck_ratio = 0.0;
    bool any_budget_exceeded = false;
};

class Profiler {
public:
    static constexpr std::size_t kCapacity = 240;

    void SetBudget(ProfilerBudget budget) noexcept { budget_ = budget; }
    [[nodiscard]] ProfilerBudget Budget() const noexcept { return budget_; }

    void Clear() noexcept {
        samples_ = {};
        cursor_ = 0;
        count_ = 0;
        snapshot_ = {};
    }

    void Observe(const ProfilerFrameStats& sample) noexcept {
        samples_[cursor_] = sample;
        cursor_ = (cursor_ + 1) % samples_.size();
        count_ = std::min<std::size_t>(count_ + 1, samples_.size());
        snapshot_ = ComputeSnapshot();
    }

    [[nodiscard]] const ProfilerSnapshot& Snapshot() const noexcept { return snapshot_; }

private:
    using ValueGetter = double (*)(const ProfilerFrameStats&);

    [[nodiscard]] ProfilerStageStats Stage(ValueGetter getter, double budget) const noexcept {
        ProfilerStageStats out;
        out.budget_ms = budget;
        if (count_ == 0) {
            return out;
        }

        std::array<double, kCapacity> values{};
        double sum = 0.0;
        for (std::size_t i = 0; i < count_; ++i) {
            const double value = std::max(0.0, getter(samples_[i]));
            values[i] = value;
            sum += value;
            out.max_ms = std::max(out.max_ms, value);
        }
        out.last_ms = getter(samples_[(cursor_ + samples_.size() - 1) % samples_.size()]);
        out.avg_ms = sum / static_cast<double>(count_);
        std::sort(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(count_));
        const std::size_t p95_index = count_ <= 1 ? 0 : static_cast<std::size_t>((static_cast<double>(count_ - 1) * 0.95) + 0.5);
        out.p95_ms = values[std::min(p95_index, count_ - 1)];
        out.over_budget = budget > 0.0 && out.p95_ms > budget;
        return out;
    }

    [[nodiscard]] ProfilerSnapshot ComputeSnapshot() const noexcept {
        ProfilerSnapshot out;
        out.sample_count = count_;
        out.total = Stage([](const ProfilerFrameStats& s) { return s.total_ms; }, budget_.total_ms);
        out.capture = Stage([](const ProfilerFrameStats& s) { return s.capture_ms; }, budget_.capture_ms);
        out.frame_pair = Stage([](const ProfilerFrameStats& s) { return s.frame_pair_ms; }, budget_.frame_pair_ms);
        out.preprocess = Stage([](const ProfilerFrameStats& s) { return s.preprocess_ms; }, budget_.preprocess_ms);
        out.inference = Stage([](const ProfilerFrameStats& s) { return s.inference_ms; }, budget_.inference_ms);
        out.onnx = Stage([](const ProfilerFrameStats& s) { return s.onnx_ms; }, budget_.onnx_ms);
        out.decode = Stage([](const ProfilerFrameStats& s) { return s.decode_ms; }, budget_.decode_ms);
        out.pipeline = Stage([](const ProfilerFrameStats& s) { return s.pipeline_ms; }, budget_.pipeline_ms);
        out.solver = Stage([](const ProfilerFrameStats& s) { return s.solver_ms; }, budget_.solver_ms);
        out.osc = Stage([](const ProfilerFrameStats& s) { return s.osc_ms; }, budget_.osc_ms);
        out.ui_publish = Stage([](const ProfilerFrameStats& s) { return s.ui_publish_ms; }, budget_.ui_publish_ms);

        const auto consider = [&](const char* name, const ProfilerStageStats& stats) {
            if (stats.budget_ms <= 0.0) {
                return;
            }
            const double ratio = stats.p95_ms / stats.budget_ms;
            if (ratio > out.bottleneck_ratio) {
                out.bottleneck_ratio = ratio;
                out.bottleneck_stage = name;
            }
            out.any_budget_exceeded = out.any_budget_exceeded || stats.over_budget;
        };
        consider("total", out.total);
        consider("capture", out.capture);
        consider("frame_pair", out.frame_pair);
        consider("preprocess", out.preprocess);
        consider("inference", out.inference);
        consider("onnx", out.onnx);
        consider("decode", out.decode);
        consider("pipeline", out.pipeline);
        consider("solver", out.solver);
        consider("osc", out.osc);
        consider("ui_publish", out.ui_publish);
        if (out.bottleneck_ratio <= 1.0 && !out.any_budget_exceeded) {
            out.bottleneck_stage = "none";
        }
        return out;
    }

    ProfilerBudget budget_{};
    std::array<ProfilerFrameStats, kCapacity> samples_{};
    std::size_t cursor_ = 0;
    std::size_t count_ = 0;
    ProfilerSnapshot snapshot_{};
};

} // namespace bt
