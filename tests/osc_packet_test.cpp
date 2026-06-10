#include "io/osc_sender.h"
#include "fakes/fake_osc_transport.h"
#include "tracking/triangulation.h"
#include "test_check.h"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

#ifdef _WIN32
using TestSocket = SOCKET;
constexpr TestSocket kInvalidTestSocket = INVALID_SOCKET;
#else
using TestSocket = int;
constexpr TestSocket kInvalidTestSocket = -1;
#endif

void CloseTestSocket(TestSocket socket) {
    if (socket == kInvalidTestSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

struct UdpSink {
    TestSocket socket = kInvalidTestSocket;
    int port = 0;

    ~UdpSink() {
        CloseTestSocket(socket);
    }

    bool Open() {
#ifdef _WIN32
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            return false;
        }
#endif
        socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket == kInvalidTestSocket) {
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = 0;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            return false;
        }
#ifdef _WIN32
        int len = sizeof(addr);
#else
        socklen_t len = sizeof(addr);
#endif
        if (::getsockname(socket, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
            return false;
        }
        port = ntohs(addr.sin_port);
        return port > 0;
    }
};

std::uint32_t ReadBe32(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return (static_cast<std::uint32_t>(data[offset]) << 24) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 8) |
           static_cast<std::uint32_t>(data[offset + 3]);
}

std::size_t PaddedStringSize(const std::string& s) {
    std::size_t n = s.size() + 1;
    while ((n % 4) != 0) {
        ++n;
    }
    return n;
}

bt::OscConfig SendableConfig(int port) {
    bt::OscConfig cfg;
    cfg.enabled = true;
    cfg.target_address = "127.0.0.1";
    cfg.target_port = port;
    cfg.send_rotations = false;
    cfg.tracker_space_transform_valid = true;
    cfg.min_confidence = 0.15f;
    return cfg;
}

bt::OscConfig FullyMappedConfig(int port) {
    bt::OscConfig cfg = SendableConfig(port);
    cfg.left_knee_tracker_index = 7;
    cfg.right_knee_tracker_index = 8;
    return cfg;
}

bt::TrackerPoseArray ValidTrackerArray(float confidence = 0.9f) {
    bt::TrackerPoseArray trackers{};
    for (std::size_t i = 0; i < bt::kTrackerRoles.size(); ++i) {
        trackers[i].role = bt::kTrackerRoles[i];
        trackers[i].valid = true;
        trackers[i].confidence = confidence;
        trackers[i].pose.position = bt::Vec3f{static_cast<float>(i), 1.0f, 2.0f};
        trackers[i].pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
        trackers[i].evidence.source = bt::TrackerEvidenceSource::DirectStereo;
        trackers[i].evidence.direct_confidence = confidence;
        trackers[i].evidence.valid = true;
    }
    return trackers;
}

} // namespace

int main() {
    const std::string address = "/tracking/trackers/1/position";
    const auto packet = bt::EncodeOscVector3MessageForTest(address, 1.0f, 2.0f, 3.0f);

    BT_CHECK(packet.size() == PaddedStringSize(address) + PaddedStringSize(",fff") + 12);
    BT_CHECK(std::string(reinterpret_cast<const char*>(packet.data())) == address);

    const std::size_t types_offset = PaddedStringSize(address);
    BT_CHECK(std::string(reinterpret_cast<const char*>(packet.data() + types_offset)) == ",fff");

    const std::size_t args = types_offset + PaddedStringSize(",fff");
    BT_CHECK(ReadBe32(packet, args + 0) == 0x3f800000u);
    BT_CHECK(ReadBe32(packet, args + 4) == 0x40000000u);
    BT_CHECK(ReadBe32(packet, args + 8) == 0x40400000u);

    const bt::Mat34f pa{{{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    const bt::Mat34f pb{{{
        1.0f, 0.0f, 0.0f, -1.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    const auto tri = bt::TriangulateLinearDLT(pa, pb, bt::Vec2f{0.5f, 0.125f}, bt::Vec2f{0.25f, 0.125f});
    BT_CHECK(tri.ok());
    BT_CHECK(tri.value().valid);
    BT_CHECK_NEAR(tri.value().world.x, 2.0, 1e-3);
    BT_CHECK_NEAR(tri.value().world.y, 0.5, 1e-3);
    BT_CHECK_NEAR(tri.value().world.z, 4.0, 1e-3);

    const auto behind = bt::TriangulateLinearDLT(pa, pb, bt::Vec2f{-0.5f, -0.125f}, bt::Vec2f{-0.25f, -0.125f});
    BT_CHECK(!behind.ok());

    bt::Mat34f bad = pa;
    bad.m[0] = std::numeric_limits<float>::quiet_NaN();
    const auto nonfinite = bt::TriangulateLinearDLT(bad, pb, bt::Vec2f{0.5f, 0.125f}, bt::Vec2f{0.25f, 0.125f});
    BT_CHECK(!nonfinite.ok());

    bt::OscConfig cfg;
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::Pelvis, cfg) == 1);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::LeftFoot, cfg) == 2);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::RightFoot, cfg) == 3);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::Chest, cfg) == 4);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::LeftElbow, cfg) == 5);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::RightElbow, cfg) == 6);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::LeftKnee, cfg) == 0);
    BT_CHECK(bt::DefaultVrchatTrackerIndex(bt::TrackerRole::RightKnee, cfg) == 0);
    bt::OscSender cache_sender(cfg);
    BT_CHECK(cache_sender.PositionAddressesForTest().size() == bt::kTrackerPoseCount);
    BT_CHECK(cache_sender.RotationAddressesForTest().size() == bt::kTrackerPoseCount);
    BT_CHECK(cache_sender.PositionAddressesForTest()[bt::TrackerRoleIndex(bt::TrackerRole::Pelvis)] == "/tracking/trackers/1/position");
    BT_CHECK(cache_sender.PositionAddressesForTest()[bt::TrackerRoleIndex(bt::TrackerRole::Chest)] == "/tracking/trackers/4/position");
    BT_CHECK(cache_sender.PositionAddressesForTest()[bt::TrackerRoleIndex(bt::TrackerRole::LeftElbow)] == "/tracking/trackers/5/position");
    BT_CHECK(cache_sender.RotationAddressesForTest()[bt::TrackerRoleIndex(bt::TrackerRole::RightElbow)] == "/tracking/trackers/6/rotation");
    BT_CHECK(cache_sender.PositionAddressesForTest()[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].empty());
    BT_CHECK(cache_sender.RotationAddressesForTest()[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)].empty());

    {
        bt::OscConfig disabled_cfg = cfg;
        disabled_cfg.enabled = false;
        bt::OscSender disabled_sender(disabled_cfg);
        const auto disabled_status = disabled_sender.SendTrackers(ValidTrackerArray());
        BT_CHECK(disabled_status.ok());
        const auto& report = disabled_sender.LastReport();
        BT_CHECK(!report.enabled);
        BT_CHECK(report.status == "disabled");
        BT_CHECK(report.sent_tracker_count == 0);
        BT_CHECK(report.sent_message_count == 0);
    }

    {
        auto bad_open_cfg = SendableConfig(9000);
        bad_open_cfg.target_address = "bad host name";
        bt::OscSender sender(bad_open_cfg);
        const auto open_status = sender.Open();
        BT_CHECK(!open_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "open_failed");
        BT_CHECK(report.last_error.find("getaddrinfo=") != std::string::npos);
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        bt::OscSender sender(FullyMappedConfig(sink.port));
        BT_CHECK(sender.Open().ok());
        const auto send_status = sender.SendTrackers(ValidTrackerArray());
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "sent");
        BT_CHECK(report.sent_tracker_count == static_cast<int>(bt::kTrackerPoseCount));
        BT_CHECK(report.skipped_tracker_count == 0);
        BT_CHECK(report.sent_message_count == static_cast<int>(bt::kTrackerPoseCount));
        for (std::size_t i = 0; i < bt::kTrackerRoles.size(); ++i) {
            BT_CHECK(report.roles[i].role == bt::kTrackerRoles[i]);
            BT_CHECK(report.roles[i].configured);
            BT_CHECK(report.roles[i].valid);
            BT_CHECK(report.roles[i].sent);
            BT_CHECK(report.roles[i].reason == "sent");
        }
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        auto low_cfg = FullyMappedConfig(sink.port);
        low_cfg.min_confidence = 0.05f;
        bt::OscSender sender(low_cfg);
        BT_CHECK(sender.Open().ok());
        const auto send_status = sender.SendTrackers(ValidTrackerArray(0.10f));
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "sent");
        BT_CHECK(report.sent_tracker_count == static_cast<int>(bt::kTrackerPoseCount));
        BT_CHECK(report.skipped_tracker_count == 0);
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        bt::OscSender sender(FullyMappedConfig(sink.port));
        BT_CHECK(sender.Open().ok());
        auto trackers = ValidTrackerArray();
        for (auto& tracker : trackers) {
            tracker.evidence.source = bt::TrackerEvidenceSource::InferredMonocular;
            tracker.evidence.stereo_fallback = true;
        }
        const auto send_status = sender.SendTrackers(trackers);
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "sent");
        BT_CHECK(report.sent_tracker_count == static_cast<int>(bt::kTrackerPoseCount));
        BT_CHECK(report.skipped_tracker_count == 0);
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        bt::OscSender sender(FullyMappedConfig(sink.port));
        BT_CHECK(sender.Open().ok());
        auto trackers = ValidTrackerArray();
        for (auto& tracker : trackers) {
            tracker.evidence.degraded = true;
        }
        trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)].evidence.source = bt::TrackerEvidenceSource::Predicted;
        trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)].evidence.direct_confidence = 0.9f;
        const auto send_status = sender.SendTrackers(trackers);
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "partial_sent");
        BT_CHECK(report.sent_tracker_count == static_cast<int>(bt::kTrackerPoseCount) - 1);
        BT_CHECK(report.skipped_tracker_count == 1);
        const auto right_knee = bt::TrackerRoleIndex(bt::TrackerRole::RightKnee);
        BT_CHECK(!report.roles[right_knee].sent);
        BT_CHECK(!report.roles[right_knee].valid);
        BT_CHECK(report.roles[right_knee].reason == "predicted_only_tracker_evidence");
        BT_CHECK(report.roles[right_knee].degraded);
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        auto skip_cfg = SendableConfig(sink.port);
        skip_cfg.chest_tracker_index = 0;
        skip_cfg.left_elbow_tracker_index = 0;
        skip_cfg.right_elbow_tracker_index = 0;
        skip_cfg.left_knee_tracker_index = 0;
        skip_cfg.right_knee_tracker_index = 0;
        bt::OscSender sender(skip_cfg);
        BT_CHECK(sender.Open().ok());
        const auto send_status = sender.SendTrackers(ValidTrackerArray());
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "partial_sent");
        BT_CHECK(report.sent_tracker_count == 3);
        BT_CHECK(report.skipped_tracker_count == 5);
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::Chest)].reason == "unmapped");
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::LeftElbow)].reason == "unmapped");
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::RightElbow)].reason == "unmapped");
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].reason == "unmapped");
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)].reason == "unmapped");
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        auto bad_index_cfg = FullyMappedConfig(sink.port);
        bad_index_cfg.left_foot_tracker_index = 9;
        bt::OscSender sender(bad_index_cfg);
        BT_CHECK(sender.Open().ok());
        const auto send_status = sender.SendTrackers(ValidTrackerArray());
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "partial_sent");
        BT_CHECK(!report.last_send_ok);
        BT_CHECK(report.sent_tracker_count == 7);
        BT_CHECK(report.skipped_tracker_count == 1);
        const auto left_foot = bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot);
        const auto right_foot = bt::TrackerRoleIndex(bt::TrackerRole::RightFoot);
        BT_CHECK(!report.roles[left_foot].sent);
        BT_CHECK(!report.roles[left_foot].valid);
        BT_CHECK(report.roles[left_foot].reason == "invalid_index_out_of_vrchat_range");
        BT_CHECK(!report.roles[left_foot].error_detail.empty());
        BT_CHECK(report.roles[right_foot].sent);
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        bt::OscSender sender(FullyMappedConfig(sink.port));
        BT_CHECK(sender.Open().ok());
        bt::test::FakeOscTransportHook fake_transport;
        fake_transport.FailAddress("/tracking/trackers/1/position", "injected pelvis send failure");
        sender.SetSendMessageHookForTest(fake_transport.Hook());
        const auto send_status = sender.SendTrackers(ValidTrackerArray());
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "partial_sent");
        BT_CHECK(!report.last_send_ok);
        BT_CHECK(report.sent_tracker_count == 7);
        BT_CHECK(report.skipped_tracker_count == 1);
        BT_CHECK(report.sent_message_count == 7);
        BT_CHECK(fake_transport.Attempts().size() == bt::kTrackerPoseCount);
        const auto pelvis = bt::TrackerRoleIndex(bt::TrackerRole::Pelvis);
        const auto right_knee = bt::TrackerRoleIndex(bt::TrackerRole::RightKnee);
        BT_CHECK(!report.roles[pelvis].sent);
        BT_CHECK(report.roles[pelvis].valid);
        BT_CHECK(report.roles[pelvis].reason == "transport_failed");
        BT_CHECK(report.roles[pelvis].error_detail == "injected pelvis send failure");
        BT_CHECK(report.roles[right_knee].sent);
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        auto mixed_cfg = FullyMappedConfig(sink.port);
        mixed_cfg.tracker_space_role_offsets[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].x =
            std::numeric_limits<float>::quiet_NaN();
        bt::OscSender sender(mixed_cfg);
        BT_CHECK(sender.Open().ok());
        sender.SetSendMessageHookForTest([](const std::string& address, float, float, float) {
            if (address == "/tracking/trackers/1/position") {
                return bt::Status::Error(bt::StatusCode::DeviceUnavailable, "injected pelvis send failure");
            }
            return bt::Status::OK();
        });
        const auto send_status = sender.SendTrackers(ValidTrackerArray());
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.status == "partial_sent");
        BT_CHECK(report.sent_tracker_count == 6);
        BT_CHECK(report.skipped_tracker_count == 2);
        const auto pelvis = bt::TrackerRoleIndex(bt::TrackerRole::Pelvis);
        const auto right_foot = bt::TrackerRoleIndex(bt::TrackerRole::RightFoot);
        BT_CHECK(report.roles[pelvis].reason == "transport_failed");
        BT_CHECK(!report.roles[pelvis].error_detail.empty());
        BT_CHECK(report.roles[right_foot].reason == "nonfinite_tracker_space_position");
        BT_CHECK(report.roles[right_foot].error_detail.empty());
    }

    {
        UdpSink sink;
        BT_CHECK(sink.Open());
        bt::OscSender sender(FullyMappedConfig(sink.port));
        BT_CHECK(sender.Open().ok());
        auto trackers = ValidTrackerArray();
        trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].valid = false;
        trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].pose.position.x = std::numeric_limits<float>::quiet_NaN();
        trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)].confidence = 0.01f;
        const auto send_status = sender.SendTrackers(trackers);
        BT_CHECK(send_status.ok());
        const auto& report = sender.LastReport();
        BT_CHECK(report.sent_tracker_count == 5);
        BT_CHECK(report.skipped_tracker_count == 3);
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)].reason == "invalid");
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::RightFoot)].reason == "nonfinite_position");
        BT_CHECK(report.roles[bt::TrackerRoleIndex(bt::TrackerRole::RightKnee)].reason == "below_min_confidence");
    }

    bt::Pose3f camera_pose;
    camera_pose.position = bt::Vec3f{1.0f, 2.0f, 3.0f};
    camera_pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    cfg.tracker_space_scale = 2.0f;
    cfg.tracker_space_position_offset = bt::Vec3f{10.0f, 20.0f, 30.0f};
    cfg.tracker_space_rotation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    const auto tracker_pose = bt::TransformCameraPoseToTrackerSpace(camera_pose, cfg);
    BT_CHECK_NEAR(tracker_pose.position.x, 12.0, 1e-5);
    BT_CHECK_NEAR(tracker_pose.position.y, 24.0, 1e-5);
    BT_CHECK_NEAR(tracker_pose.position.z, 36.0, 1e-5);
    return 0;
}
