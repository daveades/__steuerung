#include "color_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

void validate_color_settings(const std::vector<ColorRange> &colorRanges, double minimumAreaPixels)
{
    if (!std::isfinite(minimumAreaPixels) || minimumAreaPixels <= 0.0)
    {
        throw std::invalid_argument("minimum_area_pixels must be finite and greater than zero");
    }
    if (colorRanges.empty())
    {
        throw std::invalid_argument("color_names must contain at least one color");
    }
    for (std::size_t colorIndex = 0; colorIndex < colorRanges.size(); ++colorIndex)
    {
        const auto &range = colorRanges[colorIndex];
        if (range.name.empty() || std::any_of(colorRanges.begin(), colorRanges.begin() + colorIndex,
            [&range](const auto &previous) { return previous.name == range.name; }))
        {
            throw std::invalid_argument("Color names must be nonempty and unique");
        }
        for (std::size_t channelIndex = 0; channelIndex < range.lowerHsv.size(); ++channelIndex)
        {
            const auto upperLimit = channelIndex == 0 ? 179 : 255;
            if (range.lowerHsv[channelIndex] < 0 || range.lowerHsv[channelIndex] > upperLimit ||
                range.upperHsv[channelIndex] < 0 || range.upperHsv[channelIndex] > upperLimit ||
                (channelIndex != 0 && range.lowerHsv[channelIndex] > range.upperHsv[channelIndex]))
            {
                throw std::invalid_argument("Invalid HSV range for color '" + range.name +
                    "': hue must be within [0, 179], saturation/value within [0, 255], with ordered saturation/value bounds");
            }
        }
    }

}

std::vector<ColorDetection> detect_regions(const cv::Mat &image,
    const std::vector<ColorRange> &colorRanges, double minimumAreaPixels)
{
    if (image.empty() || image.type() != CV_8UC3)
    {
        throw std::invalid_argument("The detector requires a nonempty 8-bit BGR image");
    }
    validate_color_settings(colorRanges, minimumAreaPixels);
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    std::vector<ColorDetection> detections;
    for (const auto &range : colorRanges)
    {
        cv::Mat mask;
        const cv::Scalar lower(range.lowerHsv[0], range.lowerHsv[1], range.lowerHsv[2]);
        const cv::Scalar upper(range.upperHsv[0], range.upperHsv[1], range.upperHsv[2]);
        if (range.lowerHsv[0] <= range.upperHsv[0])
        {
            cv::inRange(hsvImage, lower, upper, mask);
        }
        else
        {
            cv::Mat secondMask;
            cv::inRange(hsvImage, lower, cv::Scalar(179, range.upperHsv[1], range.upperHsv[2]), mask);
            cv::inRange(hsvImage, cv::Scalar(0, range.lowerHsv[1], range.lowerHsv[2]), upper, secondMask);
            cv::bitwise_or(mask, secondMask, mask);
        }
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours)
        {
            const double areaPixels = cv::contourArea(contour);
            if (areaPixels < minimumAreaPixels)
            {
                continue;
            }
            const auto moments = cv::moments(contour);
            if (moments.m00 > 0.0)
            {
                detections.push_back({range.name,
                    {moments.m10 / moments.m00, moments.m01 / moments.m00},
                    cv::boundingRect(contour), areaPixels});
            }
        }
    }
    std::sort(detections.begin(), detections.end(), [](const auto &left, const auto &right) {
        return std::tie(left.center.y, left.center.x, left.color) <
            std::tie(right.center.y, right.center.x, right.color);
    });
    return detections;
}

cv::Mat annotate_regions(const cv::Mat &image, const std::vector<ColorDetection> &detections)
{
    auto annotatedImage = image.clone();
    for (std::size_t index = 0; index < detections.size(); ++index)
    {
        const auto &detection = detections[index];
        const cv::Point center(cvRound(detection.center.x), cvRound(detection.center.y));
        const cv::Point labelPosition(detection.bounds.x, std::max(14, detection.bounds.y - 5));
        const auto label = std::to_string(index + 1) + ": " + detection.color;
        cv::rectangle(annotatedImage, detection.bounds, cv::Scalar(0, 0, 0), 3);
        cv::rectangle(annotatedImage, detection.bounds, cv::Scalar(255, 255, 255), 1);
        cv::drawMarker(annotatedImage, center, cv::Scalar(0, 0, 0), cv::MARKER_CROSS, 13, 3);
        cv::drawMarker(annotatedImage, center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 13, 1);
        cv::putText(annotatedImage, label, labelPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
        cv::putText(annotatedImage, label, labelPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    return annotatedImage;
}
