#include "calibration/stereo_calibrator.h"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace bt {
namespace {

std::vector<cv::Point3f> BuildChessboardObjectPoints(cv::Size board_size, float square_size_meters) {
    std::vector<cv::Point3f> object_template;
    object_template.reserve(static_cast<std::size_t>(board_size.width * board_size.height));
    for (int y = 0; y < board_size.height; ++y) {
        for (int x = 0; x < board_size.width; ++x) {
            object_template.emplace_back(
                static_cast<float>(x) * square_size_meters,
                static_cast<float>(y) * square_size_meters,
                0.0f);
        }
    }
    return object_template;
}

cv::Mat CameraMatrixToCv(const CameraCalibration& camera) {
    cv::Mat k(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            k.at<double>(r, c) = camera.camera_matrix[static_cast<std::size_t>(3 * r + c)];
        }
    }
    return k;
}

cv::Mat DistortionToCv(const CameraCalibration& camera) {
    cv::Mat d(5, 1, CV_64F);
    for (int i = 0; i < 5; ++i) {
        d.at<double>(i, 0) = camera.distortion[static_cast<std::size_t>(i)];
    }
    return d;
}

Mat34f ProjectionFromKrT(const cv::Mat& k, const cv::Mat& r, const cv::Mat& t) {
    cv::Mat rt = cv::Mat::zeros(3, 4, CV_64F);
    r.copyTo(rt(cv::Rect(0, 0, 3, 3)));
    t.copyTo(rt(cv::Rect(3, 0, 1, 3)));
    const cv::Mat p = k * rt;

    Mat34f out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            out.m[static_cast<std::size_t>(4 * row + col)] = static_cast<float>(p.at<double>(row, col));
        }
    }
    return out;
}

Mat34f TransformFromRt(const cv::Mat& r, const cv::Mat& t) {
    Mat34f out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.m[static_cast<std::size_t>(4 * row + col)] = static_cast<float>(r.at<double>(row, col));
        }
        out.m[static_cast<std::size_t>(4 * row + 3)] = static_cast<float>(t.at<double>(row, 0));
    }
    return out;
}

bool IntrinsicsReady(const CameraCalibration& camera) {
    return camera.intrinsics_valid;
}

} // namespace

Result<CalibrationBundle> CalibrateStereoExtrinsicsFromChessboardObservations(
    const CalibrationBundle& input,
    cv::Size image_size,
    cv::Size board_size,
    float square_size_meters,
    const std::vector<StereoChessboardObservation>& observations) {

    if (!IntrinsicsReady(input.camera_a) || !IntrinsicsReady(input.camera_b)) {
        return Status::Error(StatusCode::FailedPrecondition, "Stereo calibration requires valid intrinsics for both cameras");
    }
    if (image_size.width <= 0 || image_size.height <= 0 || board_size.width <= 1 || board_size.height <= 1 || square_size_meters <= 0.0f) {
        return Status::Error(StatusCode::InvalidArgument, "Invalid stereo calibration image size, board size, or square size");
    }
    if (observations.size() < 5) {
        return Status::Error(StatusCode::ValidationError, "Stereo calibration needs at least five paired chessboard observations");
    }

    const std::size_t expected_points = static_cast<std::size_t>(board_size.width * board_size.height);
    const auto object_template = BuildChessboardObjectPoints(board_size, square_size_meters);
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points_a;
    std::vector<std::vector<cv::Point2f>> image_points_b;

    for (const auto& obs : observations) {
        if (obs.camera_a_points.size() != expected_points || obs.camera_b_points.size() != expected_points) {
            return Status::Error(StatusCode::ValidationError, "Stereo observation point count does not match board size");
        }
        object_points.push_back(object_template);
        image_points_a.push_back(obs.camera_a_points);
        image_points_b.push_back(obs.camera_b_points);
    }

    cv::Mat k1 = CameraMatrixToCv(input.camera_a);
    cv::Mat d1 = DistortionToCv(input.camera_a);
    cv::Mat k2 = CameraMatrixToCv(input.camera_b);
    cv::Mat d2 = DistortionToCv(input.camera_b);
    cv::Mat r;
    cv::Mat t;
    cv::Mat e;
    cv::Mat f;

    const double rms = cv::stereoCalibrate(
        object_points,
        image_points_a,
        image_points_b,
        k1,
        d1,
        k2,
        d2,
        image_size,
        r,
        t,
        e,
        f,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-6));

    if (!std::isfinite(rms) || rms > 3.0) {
        return Status::Error(StatusCode::ValidationError, "Stereo calibration reprojection RMS is too large");
    }

    CalibrationBundle out = input;
    out.camera_a.extrinsics_valid = true;
    out.camera_b.extrinsics_valid = true;

    const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    const cv::Mat zero_t = cv::Mat::zeros(3, 1, CV_64F);

    out.camera_a.world_from_camera = TransformFromRt(identity, zero_t);
    out.camera_a.image_from_world = ProjectionFromKrT(k1, identity, zero_t);

    out.camera_b.image_from_world = ProjectionFromKrT(k2, r, t);
    const cv::Mat r_inv = r.t();
    const cv::Mat t_inv = -r_inv * t;
    out.camera_b.world_from_camera = TransformFromRt(r_inv, t_inv);

    return out;
}

} // namespace bt
