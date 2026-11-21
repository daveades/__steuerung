#include "camera_undistortion.hpp"

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

static const std::string calibrationYaml =
    "%YAML:1.0\n"
    "distortion_model: plumb_bob\n"
    "image_width: 128\n"
    "image_height: 96\n"
    "camera_matrix: !!opencv-matrix\n"
    "  rows: 3\n"
    "  cols: 3\n"
    "  dt: d\n"
    "  data: [80.,0.,64.,0.,80.,48.,0.,0.,1.]\n"
    "distortion_coefficients: [0.,0.,0.,0.,0.]\n";

static cv::Mat gradient_image()
{
    cv::Mat image(96, 128, CV_8UC3);
    for (int y = 0; y < image.rows; ++y)
    {
        for (int x = 0; x < image.cols; ++x)
        {
            image.at<cv::Vec3b>(y, x) = {static_cast<unsigned char>(2 * x),
                static_cast<unsigned char>(2 * y), 100};
        }
    }
    return image;
}

class CameraUndistortionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        char fileTemplate[] = "/tmp/steuerung_camera_XXXXXX";
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

    void replace_field(const std::string &original, const std::string &replacement)
    {
        auto contents = calibrationYaml;
        const auto position = contents.find(original);
        ASSERT_NE(position, std::string::npos);
        contents.replace(position, original.size(), replacement);
        write_file(contents);
    }

    std::string filePath;
};

TEST_F(CameraUndistortionTest, ZeroDistortionPreservesPixelsDimensionsAndInput)
{
    CameraUndistortion correction(filePath);
    const auto image = gradient_image();
    const auto original = image.clone();
    const auto corrected = correction.correct_frame(image);
    EXPECT_EQ(corrected.size(), image.size());
    EXPECT_EQ(corrected.type(), image.type());
    EXPECT_EQ(cv::norm(corrected, original, cv::NORM_INF), 0.0);
    EXPECT_EQ(cv::norm(image, original, cv::NORM_INF), 0.0);
    EXPECT_NE(corrected.data, image.data);
}

TEST_F(CameraUndistortionTest, NonzeroDistortionMatchesIndependentPinholeCalculation)
{
    replace_field("[0.,0.,0.,0.,0.]", "[0.2,-0.03,0.01,-0.015,0.02]");
    CameraUndistortion correction(filePath);
    const auto image = gradient_image();
    const auto original = image.clone();
    const auto corrected = correction.correct_frame(image);
    for (const cv::Point point : {cv::Point(105, 75), {22, 22}, {95, 30}})
    {
        const double x = (point.x - 64.0) / 80.0;
        const double y = (point.y - 48.0) / 80.0;
        const double radiusSquared = x * x + y * y;
        const double radial = 1.0 + 0.2 * radiusSquared - 0.03 * radiusSquared * radiusSquared +
            0.02 * radiusSquared * radiusSquared * radiusSquared;
        const double sourceX = 80.0 * (x * radial + 0.02 * x * y -
            0.015 * (radiusSquared + 2.0 * x * x)) + 64.0;
        const double sourceY = 80.0 * (y * radial + 0.01 * (radiusSquared + 2.0 * y * y) -
            0.03 * x * y) + 48.0;
        const auto pixel = corrected.at<cv::Vec3b>(point);
        EXPECT_NEAR(pixel[0], 2.0 * sourceX, 1.0);
        EXPECT_NEAR(pixel[1], 2.0 * sourceY, 1.0);
        EXPECT_EQ(pixel[2], 100);
    }
    EXPECT_GT(cv::norm(corrected, image, cv::NORM_INF), 1.0);
    EXPECT_EQ(cv::norm(image, original, cv::NORM_INF), 0.0);
}

TEST_F(CameraUndistortionTest, AcceptsFourDistortionCoefficients)
{
    replace_field("[0.,0.,0.,0.,0.]", "[0.,0.,0.,0.]");
    CameraUndistortion correction(filePath);
    const auto image = gradient_image();
    EXPECT_EQ(cv::norm(correction.correct_frame(image), image, cv::NORM_INF), 0.0);
}

TEST_F(CameraUndistortionTest, ReusesGeometryWithoutReusingPreviousFramePixels)
{
    CameraUndistortion correction(filePath);
    const auto first = correction.correct_frame(gradient_image());
    const auto originalFirst = first.clone();
    const cv::Mat next(96, 128, CV_8UC3, cv::Scalar(11, 22, 33));
    const auto second = correction.correct_frame(next);
    EXPECT_EQ(cv::norm(second, next, cv::NORM_INF), 0.0);
    EXPECT_EQ(cv::norm(first, originalFirst, cv::NORM_INF), 0.0);
}

TEST_F(CameraUndistortionTest, RejectsIncorrectFrameSizeTypeAndEmptyInput)
{
    CameraUndistortion correction(filePath);
    EXPECT_THROW(correction.correct_frame({}), std::invalid_argument);
    EXPECT_THROW(correction.correct_frame(cv::Mat(96, 128, CV_8UC1)), std::invalid_argument);
    EXPECT_THROW(correction.correct_frame(cv::Mat(96, 128, CV_32FC3)), std::invalid_argument);
    EXPECT_THROW(correction.correct_frame(cv::Mat(48, 64, CV_8UC3)), std::invalid_argument);
    EXPECT_THROW(correction.correct_frame(cv::Mat(128, 96, CV_8UC3)), std::invalid_argument);
    EXPECT_NO_THROW(correction.correct_frame(gradient_image()));
    EXPECT_THROW(correction.correct_frame(cv::Mat(96, 160, CV_8UC3)), std::invalid_argument);
}

TEST_F(CameraUndistortionTest, RejectsUnsupportedOrMissingDistortionModel)
{
    for (const std::string replacement : {"distortion_model: equidistant", "distortion_model: unknown", ""})
    {
        replace_field("distortion_model: plumb_bob", replacement);
        EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
    }
}

TEST_F(CameraUndistortionTest, RejectsInvalidCalibrationDimensions)
{
    for (const auto &[original, replacement] : std::vector<std::pair<std::string, std::string>>{
        {"image_width: 128", "image_width: 0"},
        {"image_width: 128", "image_width: 32767"},
        {"image_height: 96", "image_height: -1"},
        {"image_height: 96", "image_height: 96.5"},
        {"image_width: 128", ""},
        {"image_height: 96", "image_height: wrong"}})
    {
        replace_field(original, replacement);
        EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
    }
}

TEST_F(CameraUndistortionTest, RejectsInvalidPinholeMatrices)
{
    for (const std::string data : {
        "[0.,0.,64.,0.,80.,48.,0.,0.,1.]",
        "[80.,0.,64.,0.,-80.,48.,0.,0.,1.]",
        "[80.,1.,64.,0.,80.,48.,0.,0.,1.]",
        "[80.,0.,64.,1.,80.,48.,0.,0.,1.]",
        "[80.,0.,64.,0.,80.,48.,1.,0.,1.]",
        "[80.,0.,64.,0.,80.,48.,0.,0.,2.]",
        "[80.,0.,.Nan,0.,80.,48.,0.,0.,1.]",
        "[.Inf,0.,64.,0.,80.,48.,0.,0.,1.]"})
    {
        replace_field("[80.,0.,64.,0.,80.,48.,0.,0.,1.]", data);
        EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
    }
    replace_field("  cols: 3", "  cols: 2");
    EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
    replace_field("camera_matrix:", "other_matrix:");
    EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
}

TEST_F(CameraUndistortionTest, RejectsInvalidDistortionCoefficients)
{
    for (const std::string coefficients : {"[]", "[0.,0.,0.]", "[0.,0.,0.,0.,0.,0.]",
        "[0.,0.,.Nan,0.,0.]", "[0.,0.,0.,.Inf,0.]", "[0.,wrong,0.,0.,0.]", "wrong"})
    {
        replace_field("[0.,0.,0.,0.,0.]", coefficients);
        EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
    }
    replace_field("distortion_coefficients:", "other_coefficients:");
    EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
}

TEST_F(CameraUndistortionTest, RejectsMissingAndMalformedFiles)
{
    EXPECT_THROW(CameraUndistortion{filePath + ".missing"}, std::invalid_argument);
    write_file("not a calibration file");
    EXPECT_THROW(CameraUndistortion{filePath}, std::invalid_argument);
}

TEST_F(CameraUndistortionTest, RejectsNonfiniteCorrectionMaps)
{
    replace_field("[0.,0.,0.,0.,0.]", "[1e308,0.,0.,0.,0.]");
    CameraUndistortion correction(filePath);
    EXPECT_THROW(correction.correct_frame(gradient_image()), std::invalid_argument);
}
