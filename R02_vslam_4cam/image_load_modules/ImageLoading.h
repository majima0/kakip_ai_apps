#pragma once

#include <mutex>
#include <atomic>
#include <memory>

#include <unistd.h>

#include <opencv2/core.hpp>

#include "Tracking.h"
#include "ImageProcessing.h"
#include "Enum.h"

namespace ORB_SLAM2 {

class Tracking;
class ImageProcessing;

class ImageLoading {
public:
    ImageLoading(const eSensor sensor);

    virtual void Run() = 0;

    void PushResult();

    void SeImageProcessor(ImageProcessing* pImageProcessor);
    ImageProcessing* mpImageProcessor;

    void RequestFinish();

    bool isFinished();

    std::unique_ptr<cv::Mat> img_ptr_;
    std::unique_ptr<cv::Mat> depth_img_ptr_;
    std::unique_ptr<cv::Mat> Tcb_ptr_;
    double timestamp_;
    std::unique_ptr<cv::Mat> camera_k_ptr_;
    std::unique_ptr<cv::Mat> dist_coef_ptr_;
    int camera_id_;

    std::atomic<bool> loaded_images;

    std::mutex mutex_frame;

protected:
    virtual void GetFrame(std::unique_ptr<cv::Mat>& img_ptr,
                          std::unique_ptr<cv::Mat>& Tcb_ptr,
                          double& timestamp,
                          std::unique_ptr<cv::Mat>& camera_k_ptr,
                          std::unique_ptr<cv::Mat>& dist_coef_ptr,
                          int& camera_id);
    virtual void GetFrame(std::unique_ptr<cv::Mat>& img_ptr,
                          std::unique_ptr<cv::Mat>& depth_img_ptr,
                          std::unique_ptr<cv::Mat>& Tcb_ptr,
                          double& timestamp,
                          std::unique_ptr<cv::Mat>& camera_k_ptr,
                          std::unique_ptr<cv::Mat>& dist_coef_ptr,
                          int& camera_id);

    size_t frame_id;

    eSensor mSensor;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
};

} // namespace ORB_SLAM2
