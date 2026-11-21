#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

struct PlaneCalibration
{
    cv::Size imageSize;
    std::vector<cv::Point2d> imagePoints;
    std::vector<cv::Point2d> robotPointsMetres;
    double surfaceZMetres = 0.0;
    double maximumFitErrorMetres = 0.001;
};

PlaneCalibration load_plane_calibration(const std::string &filePath);

class PixelToRobot
{
public:
    explicit PixelToRobot(const PlaneCalibration &calibration);
    cv::Point3d robot_point(const cv::Point2d &pixel, cv::Size imageSize) const;

private:
    cv::Size calibratedImageSize;
    cv::Matx33d mapping;
    std::vector<cv::Point2f> calibratedRegion;
    double surfaceZMetres;
};
