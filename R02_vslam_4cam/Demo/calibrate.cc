#include "calibrate.h"

#include <iostream>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <opencv2/opencv.hpp>

ChessboardInfo::ChessboardInfo(const cv::FileNode& node) {
    rows = node["rows"];
    cols = node["cols"];
    cellSize = node["cellSize"];
}

ChessboardInfo ChessboardInfo::fromFile(const char* filename) {
    ChessboardInfo info;

    cv::FileStorage fsSettings(filename, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        info.rows = 0;
        info.cols = 0;
    } else {
        info.rows = fsSettings["rows"];
        info.cols = fsSettings["cols"];
        info.cellSize = fsSettings["cellSize"];
    }
    return info;
}

std::vector<cv::Point3f> ChessboardInfo::get3dPoints() const {
    std::vector<cv::Point3f> object;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            object.push_back(cv::Point3f(i * cellSize, j * cellSize, 0.0));
    return object;
}

std::string CameraInfo::getVideoCaptureSrc() const {
    //std::stringstream ss;
    //ss << "v4l2src device=" << device << " ! "
    //   << "video/x-raw, format=YUY2, "
    //   << "width=" << width << ", height=" << height 
    //   << " ! appsink";
    //return ss.str();
    return device;
}

CameraPairCapture::CameraPairCapture(const CameraInfo& info1, const CameraInfo& info2)
: cameraInfo1(info1), cameraInfo2(info2) {
    capture1.open(cameraInfo1.getVideoCaptureSrc());
    capture2.open(cameraInfo2.getVideoCaptureSrc());
    captureThread1 = new std::thread(&CameraPairCapture::runCaptureThread, this, 0);
    captureThread2 = new std::thread(&CameraPairCapture::runCaptureThread, this, 1);
}

CameraPairCapture::~CameraPairCapture() {
    terminating = true;
    captureThread1->join();
    delete captureThread1;
    captureThread2->join();
    delete captureThread2;
    capture1.release();
    capture2.release();
}

void CameraPairCapture::runCaptureThread(int index) {
    cv::VideoCapture& cap = (index==0) ? capture1 : capture2;
    cv::Mat& frame = (index==0) ? lastFrame1 : lastFrame2;
    double& ts = (index==0) ? lastTimestamp1 : lastTimestamp2;
    double& tsOther = (index==0) ? lastTimestamp2 : lastTimestamp1;
    double tsPrev, dt1, dt2;
    bool adjust;

    while(!terminating) {
        {
            std::unique_lock<std::mutex> lock(syncMutex);

            tsPrev = ts;

            bool ok = cap.read(frame);
            ts = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();

            dt1 = ts - tsOther;
            dt2 = ts - tsPrev;

            if(!ok)
                ts = -1.;
            adjust = ts > 0 && tsPrev > 0 && tsOther > 0;
            captured = true;
        }
        int sleepTime = 100;
        if(adjust) {
            double dt3 = dt2 - dt1;
            if(dt3 > 0. && dt3 > dt2 * 0.25 && dt3 < dt2 * 0.75) {
                sleepTime += (int)(dt2 * 0.4 * 1000000);
            }
        }
        usleep(sleepTime);
    }
}

bool CameraPairCapture::readIfAvailable(cv::Mat& img1, cv::Mat& img2) {
    double ts1, ts2, dt;
    {
        std::unique_lock<std::mutex> lock(syncMutex);

        if(!captured)
            return false;

        ts1 = lastTimestamp1;
        ts2 = lastTimestamp2;

        if(ts1 <= 0 || ts2 <= 0)
            return false;

        dt = std::fabs(ts1 - ts2);
        if(dt < allowance) {
            img1 = lastFrame1;
            img2 = lastFrame2;
            captured = false;
            return true;
        } else {
            return false;
        }
    }
}

void CameraPairCapture::read(cv::Mat& img1, cv::Mat& img2) {
    while(true) {
        if(readIfAvailable(img1, img2))
            return;
        usleep(1000);
    }
}

bool findNewCorners(const ChessboardInfo& info, const cv::Mat& img, std::vector<cv::Point2f>& corners, float min_dist=20.) {
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK; 
    const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001);

    std::vector<cv::Point2f> prevCorners;
    if(!corners.empty()) {
        prevCorners = corners;
        corners.clear();
    }

    bool ok = cv::findChessboardCorners(img, info.size(), corners, flags);
    if(!ok) {
        return false;
    }

    if(corners.size() == prevCorners.size()) {
        bool moved = true;
        for(int i=0; i<corners.size(); i++) {
            float dist = cv::norm(corners[i] - prevCorners[i]);
            if(dist < min_dist) {
                moved = false;
                break;
            }
        }
        if(!moved) {
            corners = prevCorners;
            return false;
        }
    }

    cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    return true;
}

double calibrateSingle(CameraInfo& info, const ChessboardInfo& chessboard,
                       unsigned int numSamples, unsigned int frameSkips,
                       bool fixPrincipalPoint, bool fixAspectRatio, bool zeroTangentDist,
                       bool fixK1, bool fixK2, bool fixK3,
                       void (*callback)(const cv::Mat&, void*), void* arg) {
    cv::VideoCapture cap;
    cap.open(info.getVideoCaptureSrc());
    if (!cap.isOpened()) {
        std::cerr << "Failed to open " << info.device << std::endl;
        return -1.0;
    }

    cv::Mat img, imgYuv;
    std::vector<cv::Point3f> object = chessboard.get3dPoints();
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;

    int flags = 0;
    if (fixPrincipalPoint)
        flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
    if (fixAspectRatio)
        flags |= cv::CALIB_FIX_ASPECT_RATIO;
    if (zeroTangentDist)
        flags |= cv::CALIB_ZERO_TANGENT_DIST;
    if (fixK1)
        flags |= cv::CALIB_FIX_K1;
    if (fixK2)
        flags |= cv::CALIB_FIX_K2;
    if (fixK3)
        flags |= cv::CALIB_FIX_K3;
    std::cout << "CALIB flags:" << flags << std::endl;

    while(imgPoints.size() < numSamples) {
        cap.read(imgYuv);
        if (imgYuv.channels() == 3) {
            cv::cvtColor(imgYuv, img, cv::COLOR_BGR2GRAY);
        } else {
            cv::cvtColor(imgYuv, img, cv::COLOR_YUV2GRAY_YUYV);
        }
    
        bool ok = findNewCorners(chessboard, img, corners, 100.);

        if(callback) {
            cv::Mat imgBgr, imgToSend;
            if (imgYuv.channels() == 3) {
                imgBgr = imgYuv.clone();
            } else {
                cv::cvtColor(imgYuv, imgBgr, cv::COLOR_YUV2BGR_YUYV);
            }

            for(auto it=imgPoints.begin(); it!=imgPoints.end(); ++it)
                drawChessboardCorners(imgBgr, chessboard.size(), *it, true);
            drawChessboardCorners(imgBgr, chessboard.size(), corners, ok);

            std::stringstream ss;
            ss << imgPoints.size() << " / " << numSamples;
            cv::putText(
                imgBgr, ss.str(), cv::Point(15, 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);

            cv::resize(imgBgr, imgToSend, cv::Size(320, 240));
            callback(imgToSend, arg);
        } else {
            std::cout << imgPoints.size() << " / " << numSamples << std::endl;
        }

        if(!ok)
            continue;

        objPoints.push_back(object);
        imgPoints.push_back(corners);

        for (int i=0; i<frameSkips; i++)
            cap.grab();
    }

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat cam_mat, dist_coefs;

    double err = cv::calibrateCamera(
        objPoints, imgPoints, img.size(),
        cam_mat, dist_coefs, rvecs, tvecs, flags);

    info.fx = cam_mat.at<double>(0, 0);
    info.fy = cam_mat.at<double>(1, 1);
    info.cx = cam_mat.at<double>(0, 2);
    info.cy = cam_mat.at<double>(1, 2);
    info.k1 = dist_coefs.at<double>(0);
    info.k2 = dist_coefs.at<double>(1);
    info.p1 = dist_coefs.at<double>(2);
    info.p2 = dist_coefs.at<double>(3);
    if(dist_coefs.total() == 5) {
        info.k3 = dist_coefs.at<double>(4);
    } else {
        info.k3 = 0.;
    }
    return err;
}

double calibrateMulti(std::vector<CameraInfo>& cameras, const ChessboardInfo& chessboard, unsigned int numSamples, std::string windowName) {
    CameraPairCapture cap(cameras[0], cameras[1]);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open " << cameras[0].device << " and " << cameras[1].device << std::endl;
        return -1.0;
    }
    cv::Mat img1, img2, imgYuv1, imgYuv2;
    cv::Mat img(std::max(cameras[0].height, cameras[1].height), cameras[0].width + cameras[1].width, CV_8U);
    std::vector<cv::Point3f> object = chessboard.get3dPoints();
    std::vector<cv::Point2f> corners1, corners2;
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints1, imgPoints2;

    while(imgPoints1.size() < numSamples) {
        cap.read(imgYuv1, imgYuv2);

        cv::cvtColor(imgYuv1, img1, cv::COLOR_YUV2GRAY_YUYV);
        bool ok1 = findNewCorners(chessboard, img1, corners1);

        cv::cvtColor(imgYuv2, img2, cv::COLOR_YUV2GRAY_YUYV);
        bool ok2 = findNewCorners(chessboard, img2, corners2);

        if(!windowName.empty()) {
            int w = cameras[0].width;
            drawChessboardCorners(img1, chessboard.size(), corners1, ok1);
            img1.copyTo(img.colRange(0, w));
            drawChessboardCorners(img2, chessboard.size(), corners2, ok2);
            img2.copyTo(img.colRange(w, w + cameras[1].width));
            cv::imshow(windowName, img);
            cv::waitKey(100);
        }
        std::cout << corners1.size() << ", " << corners2.size() << std::endl;

        if(!ok1 || !ok2)
            continue;

        objPoints.push_back(object);
        imgPoints1.push_back(corners1);
        imgPoints2.push_back(corners2);

        std::cout << imgPoints1.size() << " / " << numSamples << std::endl;
    }

    cv::Mat K1(3, 3, CV_64F), K2(3, 3, CV_64F);
    K1.at<double>(0, 0) = cameras[0].fx;
    K1.at<double>(1, 1) = cameras[0].fy;
    K1.at<double>(0, 2) = cameras[0].cx;
    K1.at<double>(1, 2) = cameras[0].cy;
    K2.at<double>(0, 0) = cameras[1].fx;
    K2.at<double>(1, 1) = cameras[1].fy;
    K2.at<double>(0, 2) = cameras[1].cx;
    K2.at<double>(1, 2) = cameras[1].cy;

    cv::Mat distCoef1(4, 1, CV_64F), distCoef2(4, 1, CV_64F);
    distCoef1.at<double>(0) = cameras[0].k1;
    distCoef1.at<double>(1) = cameras[0].k2;
    distCoef1.at<double>(2) = cameras[0].p1;
    distCoef1.at<double>(3) = cameras[0].p2;
    if(cameras[0].k3 != 0.) {
        distCoef1.resize(5);
        distCoef1.at<double>(4) = cameras[0].k3;
    }
    distCoef2.at<double>(0) = cameras[1].k1;
    distCoef2.at<double>(1) = cameras[1].k2;
    distCoef2.at<double>(2) = cameras[1].p1;
    distCoef2.at<double>(3) = cameras[1].p2;
    if(cameras[1].k3 != 0.) {
        distCoef2.resize(5);
        distCoef2.at<double>(4) = cameras[1].k3;
    }

    cv::Mat R, T, E, F;
    double err = cv::stereoCalibrate(
        objPoints, imgPoints1, imgPoints2,
        K1, distCoef1, K2, distCoef2,
        cv::Size(cameras[0].width, cameras[0].height),
        R, T, E, F);

    // TODO: deal with cameras[0].pose
    R.copyTo(cameras[1].pose.rowRange(0,3).colRange(0,3));
    T.copyTo(cameras[1].pose.rowRange(0,3).col(3));
    return err;
}