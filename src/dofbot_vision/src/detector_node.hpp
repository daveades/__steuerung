#pragma once

#include "color_detector.hpp"

#include <rclcpp/rclcpp.hpp>

struct DetectorSettings
{
    std::vector<ColorRange> colorRanges;
    double minimumAreaPixels;
};

DetectorSettings read_detector_settings(rclcpp::Node &node);

void log_detections(const rclcpp::Logger &logger, const std::vector<ColorDetection> &detections);
