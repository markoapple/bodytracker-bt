#include "calibration/calibration_types.h"
#include "test_check.h"

#include <cmath>

int main() {
    bt::FloorPlane flat;
    flat.valid = true;
    flat.normal = bt::Vec3f{0.0f, 1.0f, 0.0f};
    flat.distance = 1.5f;

    BT_CHECK(bt::FloorPlaneUsable(flat));
    BT_CHECK_NEAR(bt::SignedDistanceToFloorPlane(bt::Vec3f{0.0f, 1.5f, 0.0f}, flat), 0.0, 1e-5);
    BT_CHECK_NEAR(bt::SignedDistanceToFloorPlane(bt::Vec3f{0.0f, 1.8f, 0.0f}, flat), 0.3, 1e-5);

    const auto projected = bt::ProjectPointToFloorPlane(bt::Vec3f{2.0f, 2.0f, -3.0f}, flat);
    BT_CHECK_NEAR(projected.x, 2.0, 1e-5);
    BT_CHECK_NEAR(projected.y, 1.5, 1e-5);
    BT_CHECK_NEAR(projected.z, -3.0, 1e-5);

    bt::FloorPlane tilted;
    tilted.valid = true;
    tilted.normal = bt::Normalize(bt::Vec3f{0.0f, 1.0f, 1.0f});
    tilted.distance = 2.0f;

    const float y = bt::FloorYAtXZOr(tilted, 0.0f, 1.0f, -99.0f);
    const bt::Vec3f point{0.0f, y, 1.0f};
    BT_CHECK(std::abs(bt::SignedDistanceToFloorPlane(point, tilted)) < 1e-4f);

    return 0;
}
