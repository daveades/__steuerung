#include "detector_node.hpp"

#include <stdexcept>

static std::array<std::int64_t, 3> read_hsv(rclcpp::Node &node, const std::string &parameterName)
{
    const auto values = node.declare_parameter<std::vector<std::int64_t>>(parameterName, std::vector<std::int64_t>{});
    if (values.size() != 3)
    {
        throw std::invalid_argument(parameterName + " must contain three integers: hue, saturation, value");
    }
    return {values[0], values[1], values[2]};
}

DetectorSettings read_detector_settings(rclcpp::Node &node)
{
    const auto colorNames = node.declare_parameter<std::vector<std::string>>("color_names", std::vector<std::string>{});
    DetectorSettings settings{{}, node.declare_parameter<double>("minimum_area_pixels", 300.0)};
    for (const auto &colorName : colorNames)
    {
        settings.colorRanges.push_back({colorName, read_hsv(node, "colors." + colorName + ".lower_hsv"),
            read_hsv(node, "colors." + colorName + ".upper_hsv")});
    }
    validate_color_settings(settings.colorRanges, settings.minimumAreaPixels);
    return settings;
}

void log_detections(const rclcpp::Logger &logger, const std::vector<ColorDetection> &detections)
{
    for (std::size_t index = 0; index < detections.size(); ++index)
    {
        const auto &detection = detections[index];
        RCLCPP_INFO(logger, "Detection %zu: color=%s, center_px=[%.2f, %.2f], size_px=[%d, %d], area_px2=%.2f",
            index + 1, detection.color.c_str(), detection.center.x, detection.center.y,
            detection.bounds.width, detection.bounds.height, detection.areaPixels);
    }
}
