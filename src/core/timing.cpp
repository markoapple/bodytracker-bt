#include "core/timing.h"

#include <chrono>

namespace bt {

QpcTimestamp NowQpc() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return QpcTimestamp{std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()};
}

double QpcDeltaSeconds(QpcTimestamp a, QpcTimestamp b) {
    return static_cast<double>(b.ticks - a.ticks) / 1000000000.0;
}

} // namespace bt
