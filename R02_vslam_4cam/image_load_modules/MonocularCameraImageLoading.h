#pragma once

#include <opencv2/videoio.hpp>

#include "ImageLoading.h"
#include "Enum.h"
#include "CameraInfo.h"

namespace ORB_SLAM2 {

class ImageLoading;

class MonocularCameraImageLoading : public ImageLoading {
public:
    MonocularCameraImageLoading(const eSensor sensor, const std::string setting_path);
    MonocularCameraImageLoading(const CameraInfo& setting, const int camera_id);

    void Run();

    void GetFrame(std::unique_ptr<cv::Mat>& img_ptr,
                  std::unique_ptr<cv::Mat>& depth_img_ptr,
                  std::unique_ptr<cv::Mat>& Tcb_ptr,
                  double& timestamp,
                  std::unique_ptr<cv::Mat>& camera_k_ptr,
                  std::unique_ptr<cv::Mat>& dist_coef_ptr,
                  int& camera_id) override;
    
    cv::Mat Tcb_;
    cv::Mat camera_k_;
    cv::Mat dist_coef_;
    int wait_ms = 0;

protected:
    cv::VideoCapture cap;
    static std::mutex shared_mutex_frame;
};

} // namespace ORB_SLAM2
