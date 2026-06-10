#pragma once

#include <cstdint>

namespace bt {

struct QpcTimestamp {
    std::int64_t ticks = 0;
};

QpcTimestamp NowQpc();
double QpcDeltaSeconds(QpcTimestamp a, QpcTimestamp b);

} // namespace bt
