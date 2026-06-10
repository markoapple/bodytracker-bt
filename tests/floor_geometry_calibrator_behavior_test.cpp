#include "calibration/floor_calibrator.h"
#include "test_check.h"

#include <cmath>
#include <vector>

namespace {

bt::FloorSeamLine2D H(float y, float x0 = 60.0f, float x1 = 580.0f, float strength = 1.0f) {
    return bt::FloorSeamLine2D{bt::Vec2f{x0, y}, bt::Vec2f{x1, y}, strength, {}};
}

bt::FloorSeamLine2D V(float x, float y0 = 170.0f, float y1 = 470.0f, float strength = 1.0f) {
    return bt::FloorSeamLine2D{bt::Vec2f{x, y0}, bt::Vec2f{x, y1}, strength, {}};
}

std::vector<bt::FloorSeamLine2D> SyntheticPlanks(float spacing_px) {
    std::vector<bt::FloorSeamLine2D> lines;
    for (int i = 0; i < 7; ++i) {
        const float y = 175.0f + spacing_px * static_cast<float>(i);
        lines.push_back(H(y, 70.0f + i * 1.5f, 575.0f - i * 2.0f, 1.0f));
    }
    return lines;
}

std::vector<bt::FloorSeamLine2D> TopOfImagePlanks(float spacing_px) {
    std::vector<bt::FloorSeamLine2D> lines;
    for (int i = 0; i < 5; ++i) {
        const float y = 32.0f + spacing_px * static_cast<float>(i);
        lines.push_back(H(y, 70.0f, 575.0f, 1.0f));
    }
    return lines;
}

std::vector<bt::FloorSeamLine2D> DirtyFloorboards(float spacing_px) {
    std::vector<bt::FloorSeamLine2D> lines;
    for (int i = 0; i < 8; ++i) {
        const float y = 165.0f + spacing_px * static_cast<float>(i) + ((i % 2) ? 1.5f : -1.0f);
        lines.push_back(H(y, 55.0f + i * 7.0f, 330.0f + i * 10.0f, 0.9f));
        if (i % 2 == 0) {
            lines.push_back(H(y + 1.0f, 365.0f, 590.0f, 0.55f));
        }
    }
    lines.push_back(H(80.0f, 20.0f, 620.0f, 3.0f));            // above floor ROI
    lines.push_back(bt::FloorSeamLine2D{bt::Vec2f{120.0f, 220.0f}, bt::Vec2f{136.0f, 236.0f}, 3.0f, {}}); // short clutter
    lines.push_back(bt::FloorSeamLine2D{bt::Vec2f{40.0f, 400.0f}, bt::Vec2f{620.0f, 445.0f}, 0.4f, {}}); // wrong angle
    return lines;
}

std::vector<bt::FloorSeamLine2D> SyntheticTiles(float y_spacing_px, float x_spacing_px) {
    auto lines = SyntheticPlanks(y_spacing_px);
    for (int i = 0; i < 6; ++i) {
        lines.push_back(V(125.0f + x_spacing_px * static_cast<float>(i), 170.0f, 470.0f, 1.0f));
    }
    return lines;
}

bt::FloorGeometryCalibrationOptions Options(const char* type) {
    bt::FloorGeometryCalibrationOptions options;
    options.floor_type = type;
    options.family_a_spacing_m = 0.20f;
    options.family_b_spacing_m = 0.30f;
    options.camera_height_m = 1.2f;
    options.horizontal_fov_deg = 70.0f;
    options.intrinsics_available = true;
    return options;
}

} // namespace

int main() {
    const auto planks = bt::EstimateFloorGeometryCalibration(SyntheticPlanks(37.5f), 640, 480, Options("planks"));
    BT_CHECK(planks.calibration.valid);
    BT_CHECK(planks.calibration.family_a.valid);
    BT_CHECK_NEAR(planks.calibration.family_a.spacing_px, 37.5f, 2.0f);
    BT_CHECK(planks.calibration.family_a.metric_spacing_valid);
    BT_CHECK(planks.calibration.metric_scale_confidence > 0.20f);
    BT_CHECK(!planks.calibration.homography_valid);
    BT_CHECK(!planks.calibration.camera_orientation_valid);
    BT_CHECK_NEAR(planks.calibration.camera_orientation_confidence, 0.0f, 1e-6f);

    const auto top_planks = bt::EstimateFloorGeometryCalibration(TopOfImagePlanks(28.0f), 640, 480, Options("planks"));
    BT_CHECK(top_planks.calibration.valid);
    BT_CHECK(top_planks.calibration.family_a.valid);
    BT_CHECK_NEAR(top_planks.calibration.family_a.spacing_px, 28.0f, 2.0f);

    const auto dirty = bt::EstimateFloorGeometryCalibration(DirtyFloorboards(36.0f), 640, 480, Options("planks"));
    BT_CHECK(dirty.calibration.valid);
    BT_CHECK(dirty.calibration.family_a.accepted_line_count >= 5);
    BT_CHECK_NEAR(dirty.calibration.family_a.spacing_px, 36.0f, 4.0f);
    BT_CHECK(dirty.calibration.family_a.rejected_line_count > 0);

    const auto tiles = bt::EstimateFloorGeometryCalibration(SyntheticTiles(40.0f, 62.0f), 640, 480, Options("tiles"));
    BT_CHECK(tiles.calibration.valid);
    BT_CHECK(tiles.calibration.family_count == 2);
    BT_CHECK(tiles.calibration.two_axis_grid_valid);
    BT_CHECK(tiles.calibration.family_a.metric_spacing_valid);
    BT_CHECK(tiles.calibration.family_b.metric_spacing_valid);
    BT_CHECK(tiles.calibration.homography_intersection_count > 0);

    std::vector<bt::FloorSeamLine2D> too_few{H(220.0f)};
    const auto rejected = bt::EstimateFloorGeometryCalibration(too_few, 640, 480, Options("planks"));
    BT_CHECK(!rejected.calibration.valid);

    return 0;
}
