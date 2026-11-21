#include "camera_undistortion.hpp"
#include "height_aware_pixel_to_robot.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>

class HeightConversion : public ::testing::Test
{
protected:
    void SetUp() override
    {
        char fileTemplate[] = "/tmp/steuerung_height_camera_XXXXXX";
        const int fileDescriptor = mkstemp(fileTemplate);
        ASSERT_GE(fileDescriptor, 0);
        close(fileDescriptor);
        filePath = fileTemplate;
        std::ofstream file(filePath);
        file << "%YAML:1.0\n"
            "distortion_model: plumb_bob\nimage_width: 640\nimage_height: 480\n"
            "camera_matrix: !!opencv-matrix\n  rows: 3\n  cols: 3\n  dt: d\n"
            "  data: [800.,0.,320.,0.,800.,240.,0.,0.,1.]\n"
            "distortion_coefficients: [0.,0.,0.,0.,0.]\n";
        file.close();
        camera = std::make_unique<CameraUndistortion>(filePath);
        const double angle = 0.35;
        rotation = {std::cos(angle), 0.0, std::sin(angle), 0.0, -1.0, 0.0,
            std::sin(angle), 0.0, -std::cos(angle)};
        translation = -(rotation * cv::Vec3d(0.0, 0.0, 0.55));
        cv::Rodrigues(rotation, rotationVector);
        calibration = {{640, 480}, {}, {{0.1, -0.08}, {0.3, -0.08}, {0.3, 0.08}, {0.1, 0.08}},
            -0.003, 0.000001};
        for (const auto &point : calibration.robotPointsMetres)
        {
            calibration.imagePoints.push_back(project({point.x, point.y, calibration.surfaceZMetres}));
        }
    }

    void TearDown() override
    {
        std::filesystem::remove(filePath);
    }

    cv::Point2d project(const cv::Point3d &point) const
    {
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(std::vector<cv::Point3d>{point}, rotationVector, translation,
            camera->camera_matrix(), cv::noArray(), pixels);
        return pixels.front();
    }

    std::string filePath;
    std::unique_ptr<CameraUndistortion> camera;
    PlaneCalibration calibration;
    cv::Matx33d rotation;
    cv::Vec3d rotationVector;
    cv::Vec3d translation;
};

TEST_F(HeightConversion, RecoversIndependentPositionsAtSeveralHeightsWithTiltedCamera)
{
    const HeightAwarePixelToRobot converter(calibration, *camera);
    for (const double height : {0.0, 0.0297, 0.1})
    {
        for (const cv::Point2d xy : {cv::Point2d(0.15, 0.03), {0.2, -0.04}, {0.25, 0.0}})
        {
            const cv::Point3d expected(xy.x, xy.y, calibration.surfaceZMetres + height);
            const auto result = converter.robot_point(project(expected), calibration.imageSize, height);
            EXPECT_NEAR(result.x, expected.x, 1e-7);
            EXPECT_NEAR(result.y, expected.y, 1e-7);
            EXPECT_DOUBLE_EQ(result.z, expected.z);
        }
    }
}

TEST_F(HeightConversion, ElevatedPointNeedsXYCorrectionAsWellAsZ)
{
    const HeightAwarePixelToRobot converter(calibration, *camera);
    const PixelToRobot tableConverter(calibration);
    const cv::Point3d expected(0.25, 0.0, 0.0267);
    const auto pixel = project(expected);
    const auto result = converter.robot_point(pixel, calibration.imageSize, 0.0297);
    const auto tablePoint = tableConverter.robot_point(pixel, calibration.imageSize);
    EXPECT_NEAR(result.x, expected.x, 1e-7);
    EXPECT_GT(std::abs(tablePoint.x - expected.x), 0.01);
}

TEST_F(HeightConversion, RejectsOutsidePhysicalWorkspaceEvenWhenPixelIsInsideImage)
{
    const HeightAwarePixelToRobot converter(calibration, *camera);
    const auto pixel = project({0.305, 0.0, 0.0267});
    ASSERT_GT(pixel.x, 0.0);
    ASSERT_LT(pixel.x, 640.0);
    EXPECT_THROW(converter.robot_point(pixel, calibration.imageSize, 0.0297), std::invalid_argument);
}

TEST_F(HeightConversion, RejectsInvalidHeightPixelAndImageSize)
{
    const HeightAwarePixelToRobot converter(calibration, *camera);
    const auto pixel = project({0.2, 0.0, 0.0267});
    for (const double height : {-0.001, 0.553, 1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(converter.robot_point(pixel, calibration.imageSize, height), std::invalid_argument);
    }
    EXPECT_THROW(converter.robot_point(pixel, {1280, 960}, 0.0297), std::invalid_argument);
    for (const cv::Point2d invalid : {cv::Point2d(-1.0, 200.0), {640.0, 200.0},
        {300.0, 480.0}, {300.0, std::numeric_limits<double>::quiet_NaN()}})
    {
        EXPECT_THROW(converter.robot_point(invalid, calibration.imageSize, 0.0297), std::invalid_argument);
    }
}

TEST_F(HeightConversion, RejectsMismatchedCalibrationAndInconsistentPinholeGeometry)
{
    auto mismatched = calibration;
    mismatched.imageSize.width = 641;
    EXPECT_THROW((HeightAwarePixelToRobot(mismatched, *camera)), std::invalid_argument);
    auto inconsistent = calibration;
    inconsistent.imagePoints[0].x += 5.0;
    EXPECT_THROW((HeightAwarePixelToRobot(inconsistent, *camera, 0.01)), std::invalid_argument);
    for (const double limit : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((HeightAwarePixelToRobot(calibration, *camera, limit)), std::invalid_argument);
    }
}
