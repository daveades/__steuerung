#include "camera_undistortion.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

CameraUndistortion::CameraUndistortion(const std::string &filePath)
{
    try
    {
        cv::FileStorage file(filePath, cv::FileStorage::READ);
        if (!file.isOpened())
        {
            throw std::invalid_argument("Could not open camera calibration: " + filePath);
        }
        if (!file["distortion_model"].isString() ||
            static_cast<std::string>(file["distortion_model"]) != "plumb_bob")
        {
            throw std::invalid_argument("Camera calibration requires distortion_model=plumb_bob");
        }
        if (!file["image_width"].isInt() || !file["image_height"].isInt())
        {
            throw std::invalid_argument("Camera calibration image dimensions must be integers");
        }
        calibratedImageSize = {static_cast<int>(file["image_width"]), static_cast<int>(file["image_height"])};
        if (calibratedImageSize.width < 2 || calibratedImageSize.height < 2 ||
            calibratedImageSize.width >= std::numeric_limits<short>::max() ||
            calibratedImageSize.height >= std::numeric_limits<short>::max())
        {
            throw std::invalid_argument("Camera calibration dimensions must be between 2 and 32766 pixels");
        }
        file["camera_matrix"] >> cameraMatrix;
        if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3 || cameraMatrix.channels() != 1)
        {
            throw std::invalid_argument("camera_matrix must be a single-channel 3-by-3 OpenCV matrix");
        }
        cameraMatrix.convertTo(cameraMatrix, CV_64F);
        if (!cv::checkRange(cameraMatrix) || cameraMatrix.at<double>(0, 0) <= 0.0 ||
            cameraMatrix.at<double>(1, 1) <= 0.0 || cameraMatrix.at<double>(0, 1) != 0.0 ||
            cameraMatrix.at<double>(1, 0) != 0.0 || cameraMatrix.at<double>(2, 0) != 0.0 ||
            cameraMatrix.at<double>(2, 1) != 0.0 || cameraMatrix.at<double>(2, 2) != 1.0)
        {
            throw std::invalid_argument("camera_matrix must contain finite pinhole intrinsics with positive focal lengths and rows [fx,0,cx], [0,fy,cy], [0,0,1]");
        }
        const auto coefficients = file["distortion_coefficients"];
        if (!coefficients.isSeq() || (coefficients.size() != 4 && coefficients.size() != 5))
        {
            throw std::invalid_argument("distortion_coefficients must contain four or five numbers: k1,k2,p1,p2[,k3]");
        }
        distortionCoefficients = cv::Mat(1, static_cast<int>(coefficients.size()), CV_64F);
        for (int index = 0; index < distortionCoefficients.cols; ++index)
        {
            const auto coefficient = coefficients[index];
            if ((!coefficient.isInt() && !coefficient.isReal()) || !std::isfinite(static_cast<double>(coefficient)))
            {
                throw std::invalid_argument("distortion_coefficients must be finite numbers");
            }
            distortionCoefficients.at<double>(0, index) = static_cast<double>(coefficient);
        }
    }
    catch (const cv::Exception &error)
    {
        throw std::invalid_argument(std::string("Invalid camera calibration file: ") + error.what());
    }
}

cv::Matx33d CameraUndistortion::camera_matrix() const
{
    return cameraMatrix;
}

cv::Size CameraUndistortion::image_size() const
{
    return calibratedImageSize;
}

cv::Mat CameraUndistortion::correct_frame(const cv::Mat &frame)
{
    if (frame.empty() || frame.type() != CV_8UC3 || frame.size() != calibratedImageSize)
    {
        throw std::invalid_argument("Frame must be an 8-bit BGR image matching the camera calibration dimensions");
    }
    if (mapX.empty())
    {
        cv::initUndistortRectifyMap(cameraMatrix, distortionCoefficients, cv::Mat(), cameraMatrix,
            calibratedImageSize, CV_32FC1, mapX, mapY);
        if (!cv::checkRange(mapX) || !cv::checkRange(mapY))
        {
            mapX.release();
            mapY.release();
            throw std::invalid_argument("Camera calibration produces nonfinite correction coordinates");
        }
    }
    cv::Mat corrected;
    cv::remap(frame, corrected, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return corrected;
}
