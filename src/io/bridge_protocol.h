#pragma once

// Wire protocol shared by the app-side SteamVR tracker bridge sender
// (src/io/steamvr_tracker_bridge.cpp) and the SteamVR driver receiver
// (src/steamvr_driver/driver_bodytracker.cpp).
//
// Both sides must agree on this layout byte for byte; the static_asserts
// below make the compiler enforce what was previously kept in sync by copy-paste.

#include <array>
#include <cstdint>

namespace bt::bridge_protocol {

constexpr std::uint32_t kMagic = 0x54535442u; // "BTST" little-endian.
constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kRoleCount = 8;
constexpr int kPort = 39560;

#pragma pack(push, 1)
struct BridgeRolePacket {
    std::uint8_t role = 0;
    std::uint8_t valid = 0;
    std::uint8_t degraded = 0;
    // 1 = role disabled by config: receiver should clear/disconnect this virtual tracker.
    std::uint8_t reserved = 0;
    float confidence = 0.0f;
    float px = 0.0f;
    float py = 0.0f;
    float pz = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
};

struct BridgePacket {
    std::uint32_t magic = kMagic;
    std::uint16_t version = kVersion;
    std::uint16_t role_count = static_cast<std::uint16_t>(kRoleCount);
    std::uint64_t sequence = 0;
    double timestamp_seconds = 0.0;
    std::array<BridgeRolePacket, kRoleCount> roles{};
};
#pragma pack(pop)

static_assert(sizeof(BridgeRolePacket) == 4 + 8 * 4, "BridgeRolePacket layout changed; bump kVersion");
static_assert(sizeof(BridgePacket) == 4 + 2 + 2 + 8 + 8 + kRoleCount * sizeof(BridgeRolePacket),
              "BridgePacket layout changed; bump kVersion");

} // namespace bt::bridge_protocol
