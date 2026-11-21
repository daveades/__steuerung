#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct ColorRange
{
    std::string name;
    std::array<std::int64_t, 3> lowerHsv;
    std::array<std::int64_t, 3> upperHsv;
};

struct ColorDetection
{
    std::string color;
    cv::Point2d center;
    cv::Rect bounds;
    double areaPixels;
};

std::vector<ColorDetection> detect_regions(const cv::Mat &image,
    const std::vector<ColorRange> &colorRanges, double minimumAreaPixels);

void validate_color_settings(const std::vector<ColorRange> &colorRanges, double minimumAreaPixels);

cv::Mat annotate_regions(const cv::Mat &image, const std::vector<ColorDetection> &detections);
