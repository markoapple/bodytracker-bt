#pragma once

#include "core/config.h"
#include "core/status.h"
#include "core/types.h"

#include <memory>

namespace bt {

class IHmdProvider {
public:
    virtual ~IHmdProvider() = default;
    virtual Result<HmdPoseSample> Poll(double target_time_seconds) = 0;
    Result<HmdPoseSample> ReadLatest() { return Poll(0.0); }
};

class NullHmdProvider final : public IHmdProvider {
public:
    Result<HmdPoseSample> Poll(double target_time_seconds) override;
};

class JsonFileHmdProvider final : public IHmdProvider {
public:
    explicit JsonFileHmdProvider(HmdProviderConfig config);
    Result<HmdPoseSample> Poll(double target_time_seconds) override;

private:
    HmdProviderConfig config_;
};


std::unique_ptr<IHmdProvider> MakeHmdProvider(const HmdProviderConfig& config);

} // namespace bt
