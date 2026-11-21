#include "height_aware_pixel_to_robot.hpp"
#include "camera_undistortion.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
struct CameraPose
{
    cv::Matx33d inverseRotation;
    cv::Vec3d origin;
    double rmsErrorPixels;
};
}

HeightAwarePixelToRobot::HeightAwarePixelToRobot(const PlaneCalibration &calibration,
    const CameraUndistortion &camera, double maximumPoseErrorPixels)
    : calibratedImageSize(calibration.imageSize), tableZMetres(calibration.surfaceZMetres),
      inverseCameraMatrix(camera.camera_matrix().inv())
{
    (void)PixelToRobot(calibration);
    if (camera.image_size() != calibratedImageSize)
    {
        throw std::invalid_argument("Camera and plane calibration dimensions must match");
    }
    if (!std::isfinite(maximumPoseErrorPixels) || maximumPoseErrorPixels <= 0.0)
    {
        throw std::invalid_argument("maximum_pose_error_pixels must be finite and positive");
    }
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2f> workspacePoints;
    for (const auto &point : calibration.robotPointsMetres)
    {
        objectPoints.emplace_back(point.x, point.y, tableZMetres);
        workspacePoints.emplace_back(point.x, point.y);
    }
    cv::convexHull(workspacePoints, calibratedRegion);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    cv::solvePnPGeneric(objectPoints, calibration.imagePoints, camera.camera_matrix(), cv::noArray(),
        rotationVectors, translationVectors, false, cv::SOLVEPNP_SQPNP);
    std::vector<CameraPose> poses;
    for (std::size_t index = 0; index < rotationVectors.size(); ++index)
    {
        cv::Matx33d rotation;
        cv::Rodrigues(rotationVectors[index], rotation);
        const cv::Vec3d translation = translationVectors[index];
        CameraPose pose{rotation.t(), -(rotation.t() * translation), 0.0};
        if (!cv::checkRange(cv::Mat(rotation)) || !cv::checkRange(cv::Mat(pose.origin)) ||
            pose.origin[2] <= tableZMetres)
        {
            continue;
        }
        std::vector<cv::Point2d> projectedPixels;
        cv::projectPoints(objectPoints, rotationVectors[index], translationVectors[index],
            camera.camera_matrix(), cv::noArray(), projectedPixels);
        bool valid = true;
        for (std::size_t pointIndex = 0; pointIndex < objectPoints.size(); ++pointIndex)
        {
            const auto &point = objectPoints[pointIndex];
            const auto cameraPoint = rotation * cv::Vec3d(point.x, point.y, point.z) + translation;
            const double error = cv::norm(projectedPixels[pointIndex] - calibration.imagePoints[pointIndex]);
            valid = valid && cameraPoint[2] > 0.0 && std::isfinite(error) && error <= maximumPoseErrorPixels;
            pose.rmsErrorPixels += error * error;
        }
        if (valid)
        {
            pose.rmsErrorPixels = std::sqrt(pose.rmsErrorPixels / objectPoints.size());
            poses.push_back(pose);
        }
    }
    if (poses.empty())
    {
        throw std::invalid_argument("No camera pose satisfies the calibration geometry and maximum_pose_error_pixels");
    }
    std::sort(poses.begin(), poses.end(), [](const auto &first, const auto &second) {
        return first.rmsErrorPixels < second.rmsErrorPixels;
    });
    if (poses.size() > 1 && poses[1].rmsErrorPixels - poses[0].rmsErrorPixels < 0.5 &&
        (cv::norm(poses[1].origin - poses[0].origin) > 0.0001 ||
            cv::norm(cv::Mat(poses[1].inverseRotation - poses[0].inverseRotation)) > 0.0001))
    {
        throw std::invalid_argument("Camera pose is ambiguous; improve the calibration view or marker measurements");
    }
    cameraToBaseRotation = poses[0].inverseRotation;
    cameraOrigin = poses[0].origin;
}

cv::Point3d HeightAwarePixelToRobot::robot_point(const cv::Point2d &pixel, cv::Size imageSize,
    double objectHeightMetres) const
{
    if (imageSize != calibratedImageSize || !std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
        pixel.x < 0.0 || pixel.y < 0.0 || pixel.x > imageSize.width - 1.0 || pixel.y > imageSize.height - 1.0)
    {
        throw std::invalid_argument("Input must use the calibrated image size and a finite pixel inside that image");
    }
    const double targetZMetres = tableZMetres + objectHeightMetres;
    if (!std::isfinite(objectHeightMetres) || objectHeightMetres < 0.0 ||
        !std::isfinite(targetZMetres) || targetZMetres >= cameraOrigin[2])
    {
        throw std::invalid_argument("Object height must be finite, nonnegative and place the surface below the camera");
    }
    const auto ray = cameraToBaseRotation * inverseCameraMatrix * cv::Vec3d(pixel.x, pixel.y, 1.0);
    if (std::abs(ray[2]) <= cv::norm(ray) * 1e-12)
    {
        throw std::invalid_argument("Camera ray is parallel to the target surface");
    }
    const double distance = (targetZMetres - cameraOrigin[2]) / ray[2];
    const auto point = cameraOrigin + distance * ray;
    if (!std::isfinite(distance) || distance <= 0.0 || !cv::checkRange(cv::Mat(point)))
    {
        throw std::invalid_argument("Surface point must be finite and in front of the camera");
    }
    if (cv::pointPolygonTest(calibratedRegion, cv::Point2d(point[0], point[1]), true) < -1e-7)
    {
        throw std::invalid_argument("Surface point lies outside the calibrated workspace");
    }
    return {point[0], point[1], targetZMetres};
}
