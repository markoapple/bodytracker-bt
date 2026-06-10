#include "tracking/geometry_cache.h"

namespace bt {

// StereoGeometryCache implementation is fully in the header.
// This translation unit exists to provide a stable link point and to
// allow future non-inline expansion (e.g. LRU eviction, multi-entry
// cache, or cache warming strategies) without changing the public API.

}  // namespace bt
