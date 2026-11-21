#pragma once

#include <opencv2/core.hpp>

#include <string>

class CameraUndistortion
{
public:
    explicit CameraUndistortion(const std::string &filePath);
    cv::Mat correct_frame(const cv::Mat &frame);
    cv::Matx33d camera_matrix() const;
    cv::Size image_size() const;

private:
    cv::Size calibratedImageSize;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    cv::Mat mapX;
    cv::Mat mapY;
};
