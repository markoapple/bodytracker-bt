#include "calibration/intrinsic_calibrator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace bt {

Result<CameraCalibration> CalibrateIntrinsicsFromChessboardFrames(
    const std::vector<cv::Mat>& frames,
    cv::Size board_size,
    float square_size_meters) {

    if (frames.empty()) {
        return Status::Error(StatusCode::InvalidArgument, "Intrinsic calibration requires at least one frame");
    }
    if (board_size.width <= 1 || board_size.height <= 1 || square_size_meters <= 0.0f) {
        return Status::Error(StatusCode::InvalidArgument, "Invalid chessboard size or square size");
    }

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

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;

    for (const auto& frame : frames) {
        if (frame.empty()) {
            continue;
        }
        if (image_size.width == 0 && image_size.height == 0) {
            image_size = frame.size();
        } else if (frame.size() != image_size) {
            return Status::Error(StatusCode::ValidationError, "Intrinsic calibration frames must all have the same image size");
        }

        cv::Mat gray;
        if (frame.channels() == 1) {
            gray = frame;
        } else {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        }

        std::vector<cv::Point2f> corners;
        const bool found = cv::findChessboardCorners(
            gray,
            board_size,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) {
            continue;
        }

        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));

        object_points.push_back(object_template);
        image_points.push_back(std::move(corners));
    }

    if (object_points.size() < 5) {
        return Status::Error(StatusCode::ValidationError, "Intrinsic calibration needs at least five detected chessboard frames");
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    const double rms = cv::calibrateCamera(
        object_points,
        image_points,
        image_size,
        camera_matrix,
        distortion,
        rvecs,
        tvecs);

    if (!std::isfinite(rms) || rms > 3.0) {
        return Status::Error(StatusCode::ValidationError, "Intrinsic calibration reprojection RMS is too large");
    }

    CameraCalibration calibration;
    calibration.intrinsics_valid = true;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            calibration.camera_matrix[static_cast<std::size_t>(3 * r + c)] = camera_matrix.at<double>(r, c);
        }
    }
    const double* distortion_values = distortion.ptr<double>(0);
    for (int i = 0; i < 5 && i < static_cast<int>(distortion.total()); ++i) {
        calibration.distortion[static_cast<std::size_t>(i)] = distortion_values[i];
    }

    return calibration;
}

} // namespace bt
