#include "pixel_to_robot.hpp"

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

static PlaneCalibration example_calibration()
{
    return {{640, 480}, {{100.0, 80.0}, {500.0, 80.0}, {500.0, 320.0}, {100.0, 320.0}},
        {{0.1, -0.1}, {0.3, -0.1}, {0.3, 0.02}, {0.1, 0.02}}, 0.025, 0.000001};
}

TEST(PixelToRobot, ConvertsIndependentPixelsAndCalibrationCornersIntoMetres)
{
    const auto calibration = example_calibration();
    const PixelToRobot converter(calibration);
    const auto centre = converter.robot_point({300.0, 200.0}, {640, 480});
    EXPECT_NEAR(centre.x, 0.2, 1e-7);
    EXPECT_NEAR(centre.y, -0.04, 1e-7);
    EXPECT_DOUBLE_EQ(centre.z, 0.025);
    const auto quarter = converter.robot_point({200.0, 140.0}, {640, 480});
    EXPECT_NEAR(quarter.x, 0.15, 1e-7);
    EXPECT_NEAR(quarter.y, -0.07, 1e-7);
    for (std::size_t index = 0; index < calibration.imagePoints.size(); ++index)
    {
        const auto point = converter.robot_point(calibration.imagePoints[index], calibration.imageSize);
        EXPECT_NEAR(point.x, calibration.robotPointsMetres[index].x, 1e-7);
        EXPECT_NEAR(point.y, calibration.robotPointsMetres[index].y, 1e-7);
    }
}

TEST(PixelToRobot, HandlesPerspectiveUsingIndependentInteriorPoints)
{
    auto calibration = example_calibration();
    const auto expectedPoint = [](const cv::Point2d &pixel) {
        const double denominator = 1.0 + 0.0005 * pixel.x + 0.0002 * pixel.y;
        return cv::Point2d((0.001 * pixel.x + 0.0002 * pixel.y + 0.05) / denominator,
            (-0.0001 * pixel.x + 0.0008 * pixel.y - 0.02) / denominator);
    };
    for (std::size_t index = 0; index < calibration.imagePoints.size(); ++index)
    {
        calibration.robotPointsMetres[index] = expectedPoint(calibration.imagePoints[index]);
    }
    const PixelToRobot converter(calibration);
    for (const cv::Point2d pixel : {cv::Point2d(200.0, 100.0), {300.0, 200.0}, {450.0, 280.0}})
    {
        const auto expected = expectedPoint(pixel);
        const auto result = converter.robot_point(pixel, calibration.imageSize);
        EXPECT_NEAR(result.x, expected.x, 1e-6);
        EXPECT_NEAR(result.y, expected.y, 1e-6);
    }
}

TEST(PixelToRobot, FitsAdditionalCorrespondences)
{
    auto calibration = example_calibration();
    calibration.imagePoints.insert(calibration.imagePoints.end(), {{300.0, 200.0}, {200.0, 140.0}});
    calibration.robotPointsMetres.insert(calibration.robotPointsMetres.end(), {{0.2, -0.04}, {0.15, -0.07}});
    const PixelToRobot converter(calibration);
    const auto result = converter.robot_point({400.0, 260.0}, calibration.imageSize);
    EXPECT_NEAR(result.x, 0.25, 1e-7);
    EXPECT_NEAR(result.y, -0.01, 1e-7);
}

TEST(PixelToRobot, RejectsInsufficientOrMismatchedCorrespondences)
{
    auto calibration = example_calibration();
    calibration.imagePoints.pop_back();
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    calibration.robotPointsMetres.pop_back();
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    calibration.imagePoints.clear();
    calibration.robotPointsMetres.clear();
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
}

TEST(PixelToRobot, RejectsDuplicatePointsInEitherCoordinateSystem)
{
    auto calibration = example_calibration();
    calibration.imagePoints[1] = calibration.imagePoints[0];
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    calibration = example_calibration();
    calibration.robotPointsMetres[1] = calibration.robotPointsMetres[0];
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
}

TEST(PixelToRobot, RejectsCollinearAndPoorlyConditionedGeometry)
{
    auto calibration = example_calibration();
    calibration.imagePoints = {{100.0, 100.0}, {200.0, 100.0}, {300.0, 100.0}, {400.0, 100.0}};
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    calibration = example_calibration();
    calibration.robotPointsMetres = {{0.1, 0.0}, {0.2, 0.0}, {0.3, 0.0}, {0.4, 0.0}};
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    calibration = example_calibration();
    calibration.imagePoints = {{100.0, 100.0}, {200.0, 100.0}, {300.0, 100.0}, {400.0, 200.0}};
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    calibration = example_calibration();
    calibration.imagePoints[2].y = 80.0 + 1e-10;
    calibration.imagePoints[3].y = 80.0 + 1e-10;
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
}

TEST(PixelToRobot, RejectsNonFiniteCalibrationValuesAndInvalidErrorLimits)
{
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
        auto calibration = example_calibration();
        calibration.imagePoints[0].x = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
        calibration = example_calibration();
        calibration.robotPointsMetres[0].y = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
        calibration = example_calibration();
        calibration.surfaceZMetres = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
        calibration = example_calibration();
        calibration.maximumFitErrorMetres = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    }
    for (const double invalid : {0.0, -0.001})
    {
        auto calibration = example_calibration();
        calibration.maximumFitErrorMetres = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    }
}

TEST(PixelToRobot, RejectsInvalidDimensionsAndCalibrationPixelsOutsideTheImage)
{
    for (const cv::Size invalid : {cv::Size(0, 480), {640, 0}, {1, 480}, {-1, 480}})
    {
        auto calibration = example_calibration();
        calibration.imageSize = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    }
    for (const cv::Point2d invalid : {cv::Point2d(-1.0, 80.0), {640.0, 80.0}, {100.0, 480.0}})
    {
        auto calibration = example_calibration();
        calibration.imagePoints[0] = invalid;
        EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
    }
}

TEST(PixelToRobot, RejectsUncalibratedPixelsAndMismatchedImageSizes)
{
    const PixelToRobot converter(example_calibration());
    for (const cv::Point2d invalid : {cv::Point2d(10.0, 10.0), {-1.0, 200.0}, {640.0, 200.0},
        {300.0, 480.0}, {std::numeric_limits<double>::quiet_NaN(), 200.0},
        {300.0, std::numeric_limits<double>::infinity()}})
    {
        EXPECT_THROW(converter.robot_point(invalid, {640, 480}), std::invalid_argument);
    }
    EXPECT_THROW(converter.robot_point({300.0, 200.0}, {1280, 960}), std::invalid_argument);
    EXPECT_THROW(converter.robot_point({300.0, 200.0}, {480, 640}), std::invalid_argument);
    EXPECT_NO_THROW(converter.robot_point({300.0, 80.0}, {640, 480}));
}

TEST(PixelToRobot, RejectsInconsistentMeasuredCorrespondences)
{
    auto calibration = example_calibration();
    calibration.imagePoints.push_back({300.0, 200.0});
    calibration.robotPointsMetres.push_back({0.21, -0.04});
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
}

TEST(PixelToRobot, RejectsAProjectiveHorizonCrossingTheCalibratedRegion)
{
    auto calibration = example_calibration();
    for (std::size_t index = 0; index < calibration.imagePoints.size(); ++index)
    {
        const auto pixel = calibration.imagePoints[index];
        calibration.robotPointsMetres[index] = {pixel.x / (pixel.x - 300.0), pixel.y / (pixel.x - 300.0)};
    }
    EXPECT_THROW(PixelToRobot{calibration}, std::invalid_argument);
}

static const std::string calibrationYaml =
    "%YAML:1.0\n"
    "frame_id: base_link\n"
    "image_coordinates: undistorted_pixels\n"
    "image_width: 640\n"
    "image_height: 480\n"
    "surface_z_metres: 0.025\n"
    "maximum_fit_error_metres: 0.000001\n"
    "image_points: [[100.0, 80.0], [500.0, 80.0], [500.0, 320.0], [100.0, 320.0]]\n"
    "robot_points_metres: [[0.1, -0.1], [0.3, -0.1], [0.3, 0.02], [0.1, 0.02]]\n";

class PlaneCalibrationFile : public ::testing::Test
{
protected:
    void SetUp() override
    {
        char fileTemplate[] = "/tmp/steuerung_plane_XXXXXX";
        const int fileDescriptor = mkstemp(fileTemplate);
        ASSERT_GE(fileDescriptor, 0);
        close(fileDescriptor);
        filePath = fileTemplate;
        write_file(calibrationYaml);
    }

    void TearDown() override
    {
        if (!filePath.empty())
        {
            std::filesystem::remove(filePath);
        }
    }

    void write_file(const std::string &contents)
    {
        std::ofstream file(filePath);
        ASSERT_TRUE(file.good());
        file << contents;
        ASSERT_TRUE(file.good());
    }

    std::string filePath;
};

TEST_F(PlaneCalibrationFile, LoadsMetadataAndConvertsAnIndependentPoint)
{
    const auto calibration = load_plane_calibration(filePath);
    EXPECT_EQ(calibration.imageSize, cv::Size(640, 480));
    ASSERT_EQ(calibration.imagePoints.size(), 4U);
    ASSERT_EQ(calibration.robotPointsMetres.size(), 4U);
    const PixelToRobot converter(calibration);
    const auto result = converter.robot_point({300.0, 200.0}, calibration.imageSize);
    EXPECT_NEAR(result.x, 0.2, 1e-7);
    EXPECT_NEAR(result.y, -0.04, 1e-7);
    EXPECT_DOUBLE_EQ(result.z, 0.025);
}

TEST_F(PlaneCalibrationFile, RejectsUnsupportedCoordinateConventions)
{
    for (const auto &[original, replacement] : std::vector<std::pair<std::string, std::string>>{
        {"frame_id: base_link", "frame_id: camera_link"},
        {"image_coordinates: undistorted_pixels", "image_coordinates: raw_pixels"}})
    {
        auto contents = calibrationYaml;
        contents.replace(contents.find(original), original.size(), replacement);
        write_file(contents);
        EXPECT_THROW(load_plane_calibration(filePath), std::invalid_argument);
    }
}

TEST_F(PlaneCalibrationFile, RejectsMissingFieldsAndWrongValueTypes)
{
    for (const auto &[original, replacement] : std::vector<std::pair<std::string, std::string>>{
        {"frame_id: base_link\n", ""},
        {"surface_z_metres: 0.025\n", ""},
        {"maximum_fit_error_metres: 0.000001\n", ""},
        {"image_width: 640", "image_width: 640.5"},
        {"surface_z_metres: 0.025", "surface_z_metres: invalid"},
        {"[100.0, 80.0]", "[100.0, 80.0, 1.0]"},
        {"[100.0, 80.0]", "[invalid, 80.0]"},
        {"image_points:", "unused_points:"}})
    {
        SCOPED_TRACE(replacement);
        auto contents = calibrationYaml;
        contents.replace(contents.find(original), original.size(), replacement);
        write_file(contents);
        EXPECT_THROW(load_plane_calibration(filePath), std::invalid_argument);
    }
}

TEST_F(PlaneCalibrationFile, RejectsMalformedAndMissingFiles)
{
    EXPECT_THROW(load_plane_calibration(filePath + ".missing"), std::invalid_argument);
    write_file("%YAML:1.0\nimage_points: [[broken\n");
    EXPECT_THROW(load_plane_calibration(filePath), std::invalid_argument);
}
