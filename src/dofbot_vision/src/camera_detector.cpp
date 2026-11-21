#include "detector_node.hpp"
#include "camera_undistortion.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

struct MeasurementView
{
    rclcpp::Logger logger;
    bool frozen = false;
    bool corrected = false;
    cv::Mat rawFrame{};
    cv::Mat imageFrame{};
    std::size_t frameIndex = 0;
    std::int64_t capturedNs = 0;
};

static void report_image_point(int event, int x, int y, int, void *userData)
{
    if (event != cv::EVENT_LBUTTONDOWN)
    {
        return;
    }
    const auto &view = *static_cast<MeasurementView *>(userData);
    if (!view.frozen)
    {
        RCLCPP_WARN(view.logger, "Pixel measurement ignored: freeze the image with Space first");
        return;
    }
    if (x < 0 || y < 0 || x >= view.imageFrame.cols || y >= view.imageFrame.rows)
    {
        return;
    }
    RCLCPP_INFO(view.logger,
        "Image point: frame=%zu, captured_ns=%lld, geometry=%s, pixel=[%d,%d], image_size=[%d,%d]",
        view.frameIndex, static_cast<long long>(view.capturedNs), view.corrected ? "corrected" : "raw",
        x, y, view.imageFrame.cols, view.imageFrame.rows);
}

static void save_measurement(const MeasurementView &view, const std::string &snapshotDirectory)
{
    if (!view.frozen)
    {
        RCLCPP_WARN(view.logger, "Snapshot ignored: freeze the image with Space first");
        return;
    }
    try
    {
        const auto directory = std::filesystem::path(snapshotDirectory) /
            ("frame_" + std::to_string(view.frameIndex) + "_" + std::to_string(view.capturedNs));
        std::filesystem::create_directories(directory.parent_path());
        if (!std::filesystem::create_directory(directory))
        {
            RCLCPP_WARN(view.logger, "Snapshot directory already exists; files preserved: %s", directory.c_str());
            return;
        }
        if (!cv::imwrite((directory / "raw.png").string(), view.rawFrame) ||
            (view.corrected && !cv::imwrite((directory / "corrected.png").string(), view.imageFrame)))
        {
            throw std::runtime_error("Could not write every snapshot image: " + directory.string());
        }
        RCLCPP_INFO(view.logger, "Snapshot saved: directory=%s, frame=%zu, image_size=[%d,%d], geometry=%s",
            directory.c_str(), view.frameIndex, view.imageFrame.cols, view.imageFrame.rows,
            view.corrected ? "raw_and_corrected" : "raw");
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(view.logger, "Snapshot save failed: %s", error.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exitCode = 1;
    bool windowOpened = false;
    MeasurementView measurement{rclcpp::get_logger("camera_detector")};
    try
    {
        auto node = std::make_shared<rclcpp::Node>("camera_detector");
        const auto cameraIndex = node->declare_parameter<std::int64_t>("camera_index", 0);
        const auto videoPath = node->declare_parameter<std::string>("video_path", "");
        const auto calibrationPath = node->declare_parameter<std::string>("camera_calibration_path", "");
        const bool showWindow = node->declare_parameter<bool>("show_window", true);
        const bool measurementMode = node->declare_parameter<bool>("measurement_mode", false);
        const auto snapshotDirectory = node->declare_parameter<std::string>(
            "snapshot_directory", "docs/calibrationCaptures");
        if (measurementMode && (!showWindow || snapshotDirectory.empty()))
        {
            throw std::invalid_argument("measurement_mode requires show_window=true and a nonempty snapshot_directory");
        }
        if (cameraIndex < 0 || cameraIndex > std::numeric_limits<int>::max())
        {
            throw std::invalid_argument("camera_index must be a nonnegative integer within the supported range");
        }
        const auto settings = read_detector_settings(*node);
        std::optional<CameraUndistortion> lensCorrection;
        if (!calibrationPath.empty())
        {
            lensCorrection.emplace(calibrationPath);
        }
        cv::VideoCapture capture;
        const bool cameraMode = videoPath.empty();
        if (!(cameraMode ? capture.open(static_cast<int>(cameraIndex), cv::CAP_V4L2) : capture.open(videoPath)))
        {
            throw std::runtime_error(cameraMode ? "Could not open camera index " + std::to_string(cameraIndex) :
                "Could not open video input: " + videoPath);
        }
        RCLCPP_INFO(node->get_logger(), "Video input opened: source=%s, display=%s, lens_correction=%s",
            cameraMode ? ("camera " + std::to_string(cameraIndex)).c_str() : videoPath.c_str(),
            showWindow ? "enabled" : "disabled", lensCorrection ? "enabled" : "disabled");
        measurement.corrected = lensCorrection.has_value();
        const auto windowName = measurementMode ? "Calibration measurements" : "Colour detections";
        if (showWindow)
        {
            cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE | (measurementMode ? cv::WINDOW_GUI_NORMAL : 0));
            windowOpened = true;
            if (measurementMode)
            {
                cv::setMouseCallback(windowName, report_image_point, &measurement);
                RCLCPP_INFO(node->get_logger(),
                    "Measurement controls enabled: Space=freeze/resume, S=save frozen image, click=read pixel, Q=quit");
            }
        }
        const double framesPerSecond = capture.get(cv::CAP_PROP_FPS);
        const int frameDelayMs = std::isfinite(framesPerSecond) && framesPerSecond > 0.0 ?
            static_cast<int>(std::clamp(1000.0 / framesPerSecond, 1.0, 1000.0)) : 33;
        std::size_t frameCount = 0;
        auto nextReportTime = std::chrono::steady_clock::time_point{};
        std::vector<ColorDetection> detections;
        std::string stopReason = "shutdown requested";
        while (rclcpp::ok())
        {
            if (!measurement.frozen)
            {
                cv::Mat frame;
                if (!capture.read(frame) || frame.empty())
                {
                    if (!rclcpp::ok())
                    {
                        break;
                    }
                    if (cameraMode || frameCount == 0)
                    {
                        throw std::runtime_error("Video frame capture failed");
                    }
                    stopReason = "video input ended";
                    break;
                }
                measurement.rawFrame = frame;
                measurement.capturedNs = node->now().nanoseconds();
                measurement.imageFrame = lensCorrection ? lensCorrection->correct_frame(frame) : frame;
                measurement.frameIndex = ++frameCount;
                detections = measurementMode ? std::vector<ColorDetection>{} :
                    detect_regions(measurement.imageFrame, settings.colorRanges, settings.minimumAreaPixels);
                const auto currentTime = std::chrono::steady_clock::now();
                if (currentTime >= nextReportTime)
                {
                    RCLCPP_INFO(node->get_logger(), "Frame processed: index=%zu, detections=%zu", frameCount, detections.size());
                    log_detections(node->get_logger(), detections);
                    nextReportTime = currentTime + std::chrono::seconds(1);
                }
            }
            if (showWindow)
            {
                cv::imshow(windowName, measurementMode ? measurement.imageFrame :
                    annotate_regions(measurement.imageFrame, detections));
                const int key = cv::waitKey(measurement.frozen ? 30 : frameDelayMs) & 0xff;
                if (measurementMode && key == ' ')
                {
                    measurement.frozen = !measurement.frozen;
                    cv::setWindowTitle(windowName, measurement.frozen ?
                        "Calibration measurements - frozen" : "Calibration measurements - live");
                    RCLCPP_INFO(node->get_logger(), "Measurement image %s: frame=%zu, captured_ns=%lld",
                        measurement.frozen ? "frozen" : "resumed", measurement.frameIndex,
                        static_cast<long long>(measurement.capturedNs));
                }
                if (measurementMode && (key == 's' || key == 'S'))
                {
                    save_measurement(measurement, snapshotDirectory);
                }
                if (key == 'q' || key == 'Q' || key == 27)
                {
                    stopReason = "quit key pressed";
                    break;
                }
                if (cv::getWindowProperty(windowName, cv::WND_PROP_VISIBLE) < 1.0)
                {
                    stopReason = "window closed";
                    break;
                }
            }
        }
        RCLCPP_INFO(node->get_logger(), "Video detection stopped: reason=%s, processed_frames=%zu, last_detections=%zu",
            stopReason.c_str(), frameCount, detections.size());
        log_detections(node->get_logger(), detections);
        exitCode = 0;
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("camera_detector"), "Video detection failed: %s", error.what());
    }
    if (windowOpened)
    {
        cv::destroyAllWindows();
    }
    rclcpp::shutdown();
    return exitCode;
}
