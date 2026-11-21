#include "detector_node.hpp"
#include "pixel_to_robot.hpp"
#include "camera_undistortion.hpp"
#include "height_aware_pixel_to_robot.hpp"

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exitCode = 1;
    try
    {
        auto node = std::make_shared<rclcpp::Node>("image_detector");
        const auto imagePath = node->declare_parameter<std::string>("image_path", "");
        const auto outputPath = node->declare_parameter<std::string>("output_path", "");
        const auto calibrationPath = node->declare_parameter<std::string>("plane_calibration_path", "");
        const auto cameraCalibrationPath = node->declare_parameter<std::string>("camera_calibration_path", "");
        const auto objectHeightMetres = node->declare_parameter<double>("object_height_metres", 0.0);
        const auto maximumPoseErrorPixels = node->declare_parameter<double>("maximum_pose_error_pixels", 3.0);
        const auto imageCoordinates = node->declare_parameter<std::string>("image_coordinates", "raw_pixels");
        if (!std::isfinite(objectHeightMetres) || objectHeightMetres < 0.0)
        {
            throw std::invalid_argument("object_height_metres must be finite and nonnegative");
        }
        if ((objectHeightMetres != 0.0 && cameraCalibrationPath.empty()) ||
            (!cameraCalibrationPath.empty() && calibrationPath.empty()))
        {
            throw std::invalid_argument("Height conversion requires camera_calibration_path and plane_calibration_path");
        }
        if (imagePath.empty() || outputPath.empty())
        {
            throw std::invalid_argument("image_path and output_path must identify the input and annotated output files");
        }
        if (std::filesystem::weakly_canonical(imagePath) == std::filesystem::weakly_canonical(outputPath) ||
            (std::filesystem::exists(imagePath) && std::filesystem::exists(outputPath) &&
                std::filesystem::equivalent(imagePath, outputPath)))
        {
            throw std::invalid_argument("output_path must differ from image_path");
        }
        if (imageCoordinates != "raw_pixels" && imageCoordinates != "undistorted_pixels")
        {
            throw std::invalid_argument("image_coordinates must be raw_pixels or undistorted_pixels");
        }
        if (!calibrationPath.empty())
        {
            if (imageCoordinates != "undistorted_pixels")
            {
                throw std::invalid_argument("Plane conversion requires image_coordinates=undistorted_pixels and an already undistorted image");
            }
            if (std::filesystem::weakly_canonical(calibrationPath) == std::filesystem::weakly_canonical(outputPath) ||
                (std::filesystem::exists(calibrationPath) && std::filesystem::exists(outputPath) &&
                    std::filesystem::equivalent(calibrationPath, outputPath)))
            {
                throw std::invalid_argument("output_path must differ from plane_calibration_path");
            }
        }
        if (!cameraCalibrationPath.empty() &&
            (std::filesystem::weakly_canonical(cameraCalibrationPath) == std::filesystem::weakly_canonical(outputPath) ||
                (std::filesystem::exists(cameraCalibrationPath) && std::filesystem::exists(outputPath) &&
                    std::filesystem::equivalent(cameraCalibrationPath, outputPath))))
        {
            throw std::invalid_argument("output_path must differ from camera_calibration_path");
        }
        const auto settings = read_detector_settings(*node);
        const auto image = cv::imread(imagePath, cv::IMREAD_COLOR);
        if (image.empty())
        {
            throw std::runtime_error("Could not read input image: " + imagePath);
        }
        std::optional<PixelToRobot> converter;
        std::optional<HeightAwarePixelToRobot> heightConverter;
        if (!calibrationPath.empty())
        {
            const auto calibration = load_plane_calibration(calibrationPath);
            if (image.size() != calibration.imageSize)
            {
                throw std::invalid_argument("Input image dimensions must match the plane calibration");
            }
            if (cameraCalibrationPath.empty())
            {
                converter.emplace(calibration);
            }
            else
            {
                const CameraUndistortion camera(cameraCalibrationPath);
                heightConverter.emplace(calibration, camera, maximumPoseErrorPixels);
                RCLCPP_INFO(node->get_logger(), "Coordinate conversion configured: mode=known_height, object_height_m=%.6f, surface_z_m=%.6f",
                    objectHeightMetres, calibration.surfaceZMetres + objectHeightMetres);
            }
        }
        const auto detections = detect_regions(image, settings.colorRanges, settings.minimumAreaPixels);
        const auto annotatedImage = annotate_regions(image, detections);
        if (!cv::imwrite(outputPath, annotatedImage))
        {
            throw std::runtime_error("Could not write annotated image: " + outputPath);
        }
        RCLCPP_INFO(node->get_logger(), "Image processed: width=%d, height=%d, detections=%zu",
            image.cols, image.rows, detections.size());
        log_detections(node->get_logger(), detections);
        if (converter || heightConverter)
        {
            for (std::size_t index = 0; index < detections.size(); ++index)
            {
                const auto &detection = detections[index];
                try
                {
                    const auto robotPoint = heightConverter ?
                        heightConverter->robot_point(detection.center, image.size(), objectHeightMetres) :
                        converter->robot_point(detection.center, image.size());
                    RCLCPP_INFO(node->get_logger(), "Detection %zu: color=%s, frame_id=base_link, surface_point_m=[%.6f, %.6f, %.6f]",
                        index + 1, detection.color.c_str(), robotPoint.x, robotPoint.y, robotPoint.z);
                }
                catch (const std::invalid_argument &error)
                {
                    RCLCPP_WARN(node->get_logger(), "Detection %zu: color=%s, coordinate conversion rejected: %s",
                        index + 1, detection.color.c_str(), error.what());
                }
            }
        }
        RCLCPP_INFO(node->get_logger(), "Annotated image saved: path=%s", outputPath.c_str());
        exitCode = 0;
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_detector"), "Image detection failed: %s", error.what());
    }
    rclcpp::shutdown();
    return exitCode;
}
