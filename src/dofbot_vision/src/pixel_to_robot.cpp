#include "pixel_to_robot.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
double read_number(const cv::FileNode &node, const std::string &name)
{
    if (!node.isInt() && !node.isReal())
    {
        throw std::invalid_argument(name + " must be numeric");
    }
    const double value = static_cast<double>(node);
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(name + " must be finite");
    }
    return value;
}

std::vector<cv::Point2d> read_points(const cv::FileNode &node, const std::string &name)
{
    if (!node.isSeq())
    {
        throw std::invalid_argument(name + " must be a list of coordinate pairs");
    }
    std::vector<cv::Point2d> points;
    for (const auto &pair : node)
    {
        if (!pair.isSeq() || pair.size() != 2)
        {
            throw std::invalid_argument(name + " must contain two numbers per point");
        }
        points.emplace_back(read_number(pair[0], name), read_number(pair[1], name));
    }
    return points;
}

bool in_image(const cv::Point2d &point, cv::Size imageSize)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        point.x >= 0.0 && point.y >= 0.0 &&
        point.x <= imageSize.width - 1.0 && point.y <= imageSize.height - 1.0;
}

std::vector<cv::Point2d> normalized_points(const std::vector<cv::Point2d> &points)
{
    cv::Point2d minimum = points.front();
    cv::Point2d maximum = points.front();
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const auto &point = points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            std::find(points.begin(), points.begin() + index, point) != points.begin() + index)
        {
            throw std::invalid_argument("Calibration points must be finite and unique in each coordinate system");
        }
        minimum.x = std::min(minimum.x, point.x);
        minimum.y = std::min(minimum.y, point.y);
        maximum.x = std::max(maximum.x, point.x);
        maximum.y = std::max(maximum.y, point.y);
    }
    const double extent = std::max(maximum.x - minimum.x, maximum.y - minimum.y);
    if (!std::isfinite(extent) || extent <= 0.0)
    {
        throw std::invalid_argument("Calibration points must span a finite region");
    }
    std::vector<cv::Point2d> normalized;
    for (const auto &point : points)
    {
        normalized.push_back((point - minimum) / extent);
    }
    return normalized;
}

cv::Point2d project_point(const cv::Matx33d &mapping, const cv::Point2d &pixel)
{
    const auto projected = mapping * cv::Vec3d(pixel.x, pixel.y, 1.0);
    const double scale = std::max({std::abs(projected[0]), std::abs(projected[1]), std::abs(projected[2])});
    if (!std::isfinite(scale) || scale == 0.0 || std::abs(projected[2]) <= scale * 1e-12)
    {
        throw std::invalid_argument("Pixel maps to an invalid projective coordinate");
    }
    const cv::Point2d result(projected[0] / projected[2], projected[1] / projected[2]);
    if (!std::isfinite(result.x) || !std::isfinite(result.y))
    {
        throw std::invalid_argument("Mapped robot coordinates must be finite");
    }
    return result;
}
}

PlaneCalibration load_plane_calibration(const std::string &filePath)
{
    try
    {
        cv::FileStorage file(filePath, cv::FileStorage::READ);
        if (!file.isOpened())
        {
            throw std::invalid_argument("Could not open plane calibration: " + filePath);
        }
        if (!file["frame_id"].isString() || static_cast<std::string>(file["frame_id"]) != "base_link" ||
            !file["image_coordinates"].isString() ||
            static_cast<std::string>(file["image_coordinates"]) != "undistorted_pixels")
        {
            throw std::invalid_argument("Calibration requires frame_id=base_link and image_coordinates=undistorted_pixels");
        }
        if (!file["image_width"].isInt() || !file["image_height"].isInt())
        {
            throw std::invalid_argument("Calibration image dimensions must be integers");
        }
        PlaneCalibration calibration;
        calibration.imageSize = {static_cast<int>(file["image_width"]), static_cast<int>(file["image_height"])};
        calibration.imagePoints = read_points(file["image_points"], "image_points");
        calibration.robotPointsMetres = read_points(file["robot_points_metres"], "robot_points_metres");
        calibration.surfaceZMetres = read_number(file["surface_z_metres"], "surface_z_metres");
        calibration.maximumFitErrorMetres = read_number(file["maximum_fit_error_metres"], "maximum_fit_error_metres");
        return calibration;
    }
    catch (const cv::Exception &error)
    {
        throw std::invalid_argument(std::string("Invalid plane calibration file: ") + error.what());
    }
}

PixelToRobot::PixelToRobot(const PlaneCalibration &calibration)
    : calibratedImageSize(calibration.imageSize), surfaceZMetres(calibration.surfaceZMetres)
{
    const auto pointCount = calibration.imagePoints.size();
    if (calibratedImageSize.width < 2 || calibratedImageSize.height < 2 ||
        pointCount < 4 || pointCount != calibration.robotPointsMetres.size() ||
        pointCount > static_cast<std::size_t>(std::numeric_limits<int>::max() / 2) ||
        !std::isfinite(surfaceZMetres) || !std::isfinite(calibration.maximumFitErrorMetres) ||
        calibration.maximumFitErrorMetres <= 0.0)
    {
        throw std::invalid_argument("Calibration needs valid dimensions, four or more paired points, a finite surface height and a positive fit-error limit");
    }
    for (const auto &point : calibration.imagePoints)
    {
        if (!in_image(point, calibratedImageSize))
        {
            throw std::invalid_argument("Calibration pixels must lie inside the calibrated image");
        }
    }
    const auto imagePoints = normalized_points(calibration.imagePoints);
    const auto robotPoints = normalized_points(calibration.robotPointsMetres);
    cv::Mat constraints = cv::Mat::zeros(static_cast<int>(pointCount * 2), 9, CV_64F);
    for (std::size_t index = 0; index < pointCount; ++index)
    {
        const double x = imagePoints[index].x;
        const double y = imagePoints[index].y;
        const double robotX = robotPoints[index].x;
        const double robotY = robotPoints[index].y;
        const std::array<double, 9> first{x, y, 1.0, 0.0, 0.0, 0.0, -robotX * x, -robotX * y, -robotX};
        const std::array<double, 9> second{0.0, 0.0, 0.0, x, y, 1.0, -robotY * x, -robotY * y, -robotY};
        std::copy(first.begin(), first.end(), constraints.ptr<double>(static_cast<int>(index * 2)));
        std::copy(second.begin(), second.end(), constraints.ptr<double>(static_cast<int>(index * 2 + 1)));
    }
    cv::Mat singularValues;
    cv::SVD::compute(constraints, singularValues);
    if (singularValues.at<double>(7) <= singularValues.at<double>(0) * 1e-9)
    {
        throw std::invalid_argument("Calibration geometry is degenerate or too poorly conditioned");
    }
    const auto homography = cv::findHomography(calibration.imagePoints, calibration.robotPointsMetres, 0);
    if (homography.empty() || !cv::checkRange(homography))
    {
        throw std::invalid_argument("Could not calculate a finite plane mapping");
    }
    cv::SVD::compute(homography, singularValues);
    if (singularValues.at<double>(2) <= singularValues.at<double>(0) * 1e-12)
    {
        throw std::invalid_argument("Plane mapping is singular or too poorly conditioned");
    }
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            mapping(row, column) = homography.at<double>(row, column);
        }
    }
    std::vector<cv::Point2f> pixels;
    for (const auto &point : calibration.imagePoints)
    {
        pixels.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
    }
    cv::convexHull(pixels, calibratedRegion);
    double minimumDenominator = std::numeric_limits<double>::infinity();
    double maximumDenominator = -std::numeric_limits<double>::infinity();
    for (const auto &point : calibratedRegion)
    {
        const double denominator = mapping(2, 0) * point.x + mapping(2, 1) * point.y + mapping(2, 2);
        minimumDenominator = std::min(minimumDenominator, denominator);
        maximumDenominator = std::max(maximumDenominator, denominator);
    }
    const double denominatorScale = std::max(std::abs(minimumDenominator), std::abs(maximumDenominator));
    if (!std::isfinite(denominatorScale) || denominatorScale == 0.0 ||
        (minimumDenominator <= denominatorScale * 1e-10 && maximumDenominator >= -denominatorScale * 1e-10))
    {
        throw std::invalid_argument("Plane mapping crosses a projective singularity inside the calibration region");
    }
    for (std::size_t index = 0; index < pointCount; ++index)
    {
        const auto mapped = project_point(mapping, calibration.imagePoints[index]);
        const auto &expected = calibration.robotPointsMetres[index];
        if (std::hypot(mapped.x - expected.x, mapped.y - expected.y) > calibration.maximumFitErrorMetres)
        {
            throw std::invalid_argument("Plane mapping exceeds maximum_fit_error_metres");
        }
    }
}

cv::Point3d PixelToRobot::robot_point(const cv::Point2d &pixel, cv::Size imageSize) const
{
    if (imageSize != calibratedImageSize || !in_image(pixel, imageSize))
    {
        throw std::invalid_argument("Input must use the calibrated image size and a finite pixel inside that image");
    }
    if (cv::pointPolygonTest(calibratedRegion, pixel, false) < 0.0)
    {
        throw std::invalid_argument("Pixel lies outside the calibrated region");
    }
    const auto result = project_point(mapping, pixel);
    return {result.x, result.y, surfaceZMetres};
}
