#pragma once

#include <opencv2/opencv.hpp>

struct CameraInfo {
    CameraInfo() {}
    CameraInfo(const cv::FileNode& node);

    std::string device;
    int width;
    int height;
    double fx=1., fy=1., cx=0., cy=0.;
    double k1=0., k2=0., k3=0., p1=0., p2=0.;
    double fps = 30.;
    double bf = 40.;
    int wait_ms = 1;
    cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);

    std::string getVideoCaptureSrc() const;
    static std::vector<CameraInfo> fromFile(const char* filename);
    static std::vector<CameraInfo> fromFile(std::string filename) {
        return fromFile(filename.c_str());
    }
    void save(const std::string& filename);
};