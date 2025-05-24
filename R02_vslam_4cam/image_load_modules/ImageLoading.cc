#include "ImageLoading.h"

#include <cassert>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

ImageLoading::ImageLoading(const eSensor sensor)
    : mpImageProcessor(NULL),
      timestamp_(0.0),
      camera_id_(0),
      frame_id(0),
      mSensor(sensor),
      mbFinishRequested(false), mbFinished(true) {
    loaded_images.store(false);
}

void ImageLoading::PushResult() {
    if (mSensor == eSensor::MONOCULAR) {
        GetFrame(mpImageProcessor->input_image_ptr_,
                 mpImageProcessor->Tcb_ptr_,
                 mpImageProcessor->timestamp_,
                 mpImageProcessor->camera_k_ptr_,
                 mpImageProcessor->dist_coef_ptr_,
                 mpImageProcessor->camera_id_);
    }
    else if (mSensor == eSensor::STEREO) {
        fprintf(stderr, "ImageLoading not support Stereo mode.\n");
        exit(EXIT_FAILURE);
    }
    else if (mSensor == eSensor::RGBD) {
        GetFrame(mpImageProcessor->input_image_ptr_,
                 mpImageProcessor->depth_image_ptr_,
                 mpImageProcessor->Tcb_ptr_,
                 mpImageProcessor->timestamp_,
                 mpImageProcessor->camera_k_ptr_,
                 mpImageProcessor->dist_coef_ptr_,
                 mpImageProcessor->camera_id_);
    }
    else {
        fprintf(stderr, "Unknown mode detected in ImageProcessing::Run.\n");
        exit(EXIT_FAILURE);
    }

    loaded_images.store(false);
    mpImageProcessor->wait_for_next_frame.store(false);
}

void ImageLoading::SeImageProcessor(ImageProcessing* pImageProcessor) {
    mpImageProcessor = pImageProcessor;
}

void ImageLoading::RequestFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ImageLoading::isFinished() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

void ImageLoading::GetFrame(
    std::unique_ptr<cv::Mat>& img_ptr,
    std::unique_ptr<cv::Mat>& Tcb_ptr,
    double& timestamp,
    std::unique_ptr<cv::Mat>& camera_k_ptr,
    std::unique_ptr<cv::Mat>& dist_coef_ptr,
    int& camera_id) {
    std::unique_lock<std::mutex> lock(mutex_frame);
    img_ptr = std::move(img_ptr_);
    Tcb_ptr = std::move(Tcb_ptr_);
    timestamp = timestamp_;
    camera_k_ptr = std::move(camera_k_ptr_);
    dist_coef_ptr = std::move(dist_coef_ptr_);
    camera_id = camera_id_;
}

void ImageLoading::GetFrame(
    std::unique_ptr<cv::Mat>& img_ptr,
    std::unique_ptr<cv::Mat>& depth_img_ptr,
    std::unique_ptr<cv::Mat>& Tcb_ptr,
    double& timestamp,
    std::unique_ptr<cv::Mat>& camera_k_ptr,
    std::unique_ptr<cv::Mat>& dist_coef_ptr,
    int& camera_id) {
    std::unique_lock<std::mutex> lock(mutex_frame);
    img_ptr = std::move(img_ptr_);
    depth_img_ptr = std::move(depth_img_ptr_);
    Tcb_ptr = std::move(Tcb_ptr_);
    timestamp = timestamp_;
    camera_k_ptr = std::move(camera_k_ptr_);
    dist_coef_ptr = std::move(dist_coef_ptr_);
    camera_id = camera_id_;
}

bool ImageLoading::CheckFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ImageLoading::SetFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

} // namespace ORB_SLAM2