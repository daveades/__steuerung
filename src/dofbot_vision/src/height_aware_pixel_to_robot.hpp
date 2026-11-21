#pragma once

#include "pixel_to_robot.hpp"

class CameraUndistortion;

class HeightAwarePixelToRobot
{
public:
    HeightAwarePixelToRobot(const PlaneCalibration &calibration,
        const CameraUndistortion &camera, double maximumPoseErrorPixels = 3.0);
    cv::Point3d robot_point(const cv::Point2d &pixel, cv::Size imageSize,
        double objectHeightMetres) const;

private:
    cv::Size calibratedImageSize;
    double tableZMetres;
    cv::Matx33d inverseCameraMatrix;
    cv::Matx33d cameraToBaseRotation;
    cv::Vec3d cameraOrigin;
    std::vector<cv::Point2f> calibratedRegion;
};
