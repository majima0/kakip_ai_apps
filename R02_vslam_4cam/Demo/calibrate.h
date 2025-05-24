#pragma once

#include <string>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>

#include <CameraInfo.h>

struct ChessboardInfo {
    ChessboardInfo() {}
    ChessboardInfo(const cv::FileNode& node);

    int cols = 9;
    int rows = 6;
    double cellSize = 0.025;

    static ChessboardInfo fromFile(const char* filename);
    static ChessboardInfo fromFile(std::string filename) {
        return fromFile(filename.c_str());
    }
    std::vector<cv::Point3f> get3dPoints() const;
    cv::Size size() const { return cv::Size(cols, rows); }
};

class CameraPairCapture {
public:
    CameraPairCapture(const CameraInfo& cameraInfo1, const CameraInfo& cameraInfo2);
    ~CameraPairCapture();

    CameraInfo cameraInfo1, cameraInfo2;
    double allowance = 0.05;
    bool readIfAvailable(cv::Mat& img1, cv::Mat& img2);
    void read(cv::Mat& img1, cv::Mat& img2);
    bool isOpened() { return capture1.isOpened() && capture2.isOpened(); }

protected:
    cv::VideoCapture capture1, capture2;
    bool captured=false;
    bool terminating=false;
    std::thread *captureThread1, *captureThread2;
    cv::Mat lastFrame1, lastFrame2;
    double lastTimestamp1=-1., lastTimestamp2=-1.;
    std::mutex syncMutex;
    void runCaptureThread(int index);
};

double calibrateSingle(
    CameraInfo& cameras,
    const ChessboardInfo& chessboard,
    unsigned int numSamples=20,
    unsigned int frameSkips=0,
    bool fixPrincipalPoint=false,
    bool fixAspectRatio=false,
    bool zeroTangentDist=false,
    bool fixK1=false,
    bool fixK2=false,
    bool fixK3=false,
    void (*callback)(const cv::Mat&, void*)=nullptr, void* arg=nullptr
);
double calibrateMulti(std::vector<CameraInfo>& cameras, const ChessboardInfo& chessboard, unsigned int numSamples=20, std::string windowName="");