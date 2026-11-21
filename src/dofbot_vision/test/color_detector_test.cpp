#include "color_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

static const std::vector<ColorRange> colorRanges{
    {"red", {170, 80, 60}, {10, 255, 255}},
    {"green", {35, 80, 60}, {85, 255, 255}},
    {"blue", {90, 80, 60}, {130, 255, 255}}};

TEST(ColorDetector, FindsSeveralColorsAndSeparateObjectsOfTheSameColor)
{
    cv::Mat image(120, 240, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(image, cv::Rect(10, 10, 40, 30), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(image, cv::Rect(70, 10, 40, 30), cv::Scalar(0, 255, 0), cv::FILLED);
    cv::rectangle(image, cv::Rect(130, 10, 40, 30), cv::Scalar(255, 0, 0), cv::FILLED);
    cv::rectangle(image, cv::Rect(10, 70, 40, 30), cv::Scalar(0, 0, 255), cv::FILLED);
    const auto detections = detect_regions(image, colorRanges, 100.0);
    ASSERT_EQ(detections.size(), 4U);
    EXPECT_EQ(detections[0].color, "red");
    EXPECT_EQ(detections[1].color, "green");
    EXPECT_EQ(detections[2].color, "blue");
    EXPECT_EQ(detections[3].color, "red");
    EXPECT_DOUBLE_EQ(detections[0].center.x, 29.5);
    EXPECT_DOUBLE_EQ(detections[0].center.y, 24.5);
    EXPECT_EQ(detections[0].bounds, cv::Rect(10, 10, 40, 30));
    EXPECT_DOUBLE_EQ(detections[0].areaPixels, 39.0 * 29.0);
}

TEST(ColorDetector, DetectsHueOnBothSidesOfTheRedBoundary)
{
    cv::Mat hsvImage(40, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(hsvImage, cv::Rect(5, 5, 30, 30), cv::Scalar(2, 255, 255), cv::FILLED);
    cv::rectangle(hsvImage, cv::Rect(55, 5, 30, 30), cv::Scalar(178, 255, 255), cv::FILLED);
    cv::Mat image;
    cv::cvtColor(hsvImage, image, cv::COLOR_HSV2BGR);
    const auto detections = detect_regions(image, {colorRanges[0]}, 100.0);
    ASSERT_EQ(detections.size(), 2U);
    EXPECT_EQ(detections[0].color, "red");
    EXPECT_EQ(detections[1].color, "red");
}

TEST(ColorDetector, FiltersSmallNoiseAndHonorsTheAreaThreshold)
{
    cv::Mat image(80, 120, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(image, cv::Rect(5, 5, 10, 10), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(image, cv::Rect(40, 20, 30, 30), cv::Scalar(0, 0, 255), cv::FILLED);
    const auto detections = detect_regions(image, {colorRanges[0]}, 100.0);
    ASSERT_EQ(detections.size(), 1U);
    EXPECT_EQ(detections[0].bounds, cv::Rect(40, 20, 30, 30));
    EXPECT_TRUE(detect_regions(image, {colorRanges[0]}, 1000.0).empty());
}

TEST(ColorDetector, UsesConfiguredColorRangesAndSaturationBounds)
{
    cv::Mat image(60, 120, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(image, cv::Rect(10, 10, 30, 30), cv::Scalar(0, 255, 255), cv::FILLED);
    EXPECT_TRUE(detect_regions(image, {colorRanges[0]}, 100.0).empty());
    const auto detections = detect_regions(image, {{"custom", {20, 100, 100}, {40, 255, 255}}}, 100.0);
    ASSERT_EQ(detections.size(), 1U);
    EXPECT_EQ(detections[0].color, "custom");
}

TEST(ColorDetector, ReturnsAnEmptyListForNoMatch)
{
    const cv::Mat image(80, 120, CV_8UC3, cv::Scalar(255, 255, 255));
    EXPECT_TRUE(detect_regions(image, colorRanges, 100.0).empty());
}

TEST(ColorDetector, RecalculatesCentersAsObjectsMoveBetweenFrames)
{
    for (int frameIndex = 0; frameIndex < 5; ++frameIndex)
    {
        cv::Mat image(80, 240, CV_8UC3, cv::Scalar(0, 0, 0));
        const int leftX = 10 + frameIndex * 30;
        cv::rectangle(image, cv::Rect(leftX, 20, 30, 30), cv::Scalar(0, 0, 255), cv::FILLED);
        const auto detections = detect_regions(image, {colorRanges[0]}, 100.0);
        ASSERT_EQ(detections.size(), 1U);
        EXPECT_DOUBLE_EQ(detections[0].center.x, leftX + 14.5);
        EXPECT_DOUBLE_EQ(detections[0].center.y, 34.5);
    }
}

TEST(ColorDetector, AnnotationMarksCentersWithoutChangingTheInput)
{
    cv::Mat image(80, 120, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(image, cv::Rect(20, 20, 40, 40), cv::Scalar(0, 0, 255), cv::FILLED);
    const auto original = image.clone();
    const auto detections = detect_regions(image, {colorRanges[0]}, 100.0);
    const auto annotated = annotate_regions(image, detections);
    EXPECT_EQ(annotated.size(), image.size());
    EXPECT_EQ(cv::norm(image, original, cv::NORM_INF), 0.0);
    EXPECT_GT(cv::norm(annotated, image, cv::NORM_INF), 0.0);
    EXPECT_EQ(annotated.at<cv::Vec3b>(40, 40), cv::Vec3b(255, 255, 255));
}

TEST(ColorDetector, RejectsInvalidImagesAndAreaThresholds)
{
    const cv::Mat image(40, 40, CV_8UC3, cv::Scalar(0, 0, 0));
    EXPECT_THROW(detect_regions({}, colorRanges, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(cv::Mat(40, 40, CV_8UC1), colorRanges, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, colorRanges, 0.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, colorRanges, -1.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, colorRanges, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, colorRanges, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(ColorDetector, RejectsInvalidNamesAndHsvBounds)
{
    const cv::Mat image(40, 40, CV_8UC3, cv::Scalar(0, 0, 0));
    EXPECT_THROW(detect_regions(image, {}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {colorRanges[0], colorRanges[0]}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {{"", {0, 0, 0}, {10, 255, 255}}}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {{"bad", {180, 0, 0}, {10, 255, 255}}}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {{"bad", {-1, 0, 0}, {10, 255, 255}}}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {{"bad", {0, 0, 0}, {10, 256, 255}}}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {{"bad", {0, 100, 0}, {10, 80, 255}}}, 100.0), std::invalid_argument);
    EXPECT_THROW(detect_regions(image, {{"bad", {0, 0, 100}, {10, 255, 80}}}, 100.0), std::invalid_argument);
}
