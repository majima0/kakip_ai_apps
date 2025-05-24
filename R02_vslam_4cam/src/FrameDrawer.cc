/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

std::map<const std::string, const cv::Scalar> col = {
    {"background", cv::Scalar(0, 0, 0)},
    {"aeroplane", cv::Scalar(255, 0, 0)},
    {"bicycle", cv::Scalar(255, 76, 0)},
    {"bird", cv::Scalar(255, 153, 0)},
    {"boat", cv::Scalar(255, 229, 0)},
    {"bottle", cv::Scalar(204, 255, 0)},
    {"bus", cv::Scalar(127, 255, 0)},
    {"car", cv::Scalar(50, 255, 0)},
    {"cat", cv::Scalar(0, 255, 25)},
    {"chair", cv::Scalar(0, 255, 101)},
    {"cow", cv::Scalar(0, 255, 178)},
    {"diningtable", cv::Scalar(0, 255, 255)},
    {"dog", cv::Scalar(0, 178, 255)},
    {"horse", cv::Scalar(0, 101, 255)},
    {"motorbike", cv::Scalar(0, 25, 255)},
    {"person", cv::Scalar(51, 178, 255)},
    {"pottedplant", cv::Scalar(127, 0, 255)},
    {"sheep", cv::Scalar(203, 0, 255)},
    {"sofa", cv::Scalar(255, 0, 229)},
    {"train", cv::Scalar(255, 0, 152)},
    {"tvmonitor", cv::Scalar(255, 0, 76)}};

FrameDrawer::FrameDrawer(Map* pMap, int nCameras)
    : mpMap(pMap), mnCameras(nCameras),
    mIm(nCameras), mvCurrentKeys(nCameras), yoloBoundingBoxList(nCameras),
    mpPlaneSegmentImg(nCameras), mvbVO(nCameras), mvbMap(nCameras),
    mvIniKeys(nCameras), mvIniMatches(nCameras) {
    mState = eTrackingState::SYSTEM_NOT_READY;
    for (int i = 0; i < mnCameras; i++) {
        mIm[i] = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::RNG rng(0xFFFFFFFF);
    for (int i = 0; i < COLORNUM; i++) {
        rng.fill(randColor[i], cv::RNG::UNIFORM, 0, 256);
    }

    // Populate with random color codes for plane segment result
    for (int i = 0; i < 100; i++) {
        cv::Vec3b color;
        color[0] = rand() % 255;
        color[1] = rand() % 255;
        color[2] = rand() % 255;
        colorCodeForPlaneSegmentResult.push_back(color);
    }
    // Add specific colors for planes
    colorCodeForPlaneSegmentResult[0][0] = 0;
    colorCodeForPlaneSegmentResult[0][1] = 0;
    colorCodeForPlaneSegmentResult[0][2] = 255;
    colorCodeForPlaneSegmentResult[1][0] = 255;
    colorCodeForPlaneSegmentResult[1][1] = 0;
    colorCodeForPlaneSegmentResult[1][2] = 204;
    colorCodeForPlaneSegmentResult[2][0] = 255;
    colorCodeForPlaneSegmentResult[2][1] = 100;
    colorCodeForPlaneSegmentResult[2][2] = 0;
    colorCodeForPlaneSegmentResult[3][0] = 0;
    colorCodeForPlaneSegmentResult[3][1] = 153;
    colorCodeForPlaneSegmentResult[3][2] = 255;
}

cv::Mat FrameDrawer::DrawSingleFrame(bool drawPlaneSegment, int cameraId) {
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys;     // Initialization: KeyPoints in reference frame
    vector<int> vMatches;              // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
    int state;                         // Tracking state

    vector<YoloBoundingBox> vYoloBoundingBox = this->yoloBoundingBoxList[cameraId];  // TODO: workaround

    // Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state = mState;
        if (mState == eTrackingState::SYSTEM_NOT_READY)
            mState = eTrackingState::NO_IMAGES_YET;

        mIm[cameraId].copyTo(im);

        if (mState == eTrackingState::NOT_INITIALIZED) {
            vCurrentKeys = mvCurrentKeys[cameraId];
            vIniKeys = mvIniKeys[cameraId];
            vMatches = mvIniMatches[cameraId];
        }
        else if (mState == eTrackingState::OK) {
            vCurrentKeys = mvCurrentKeys[cameraId];
            vbVO = mvbVO[cameraId];
            vbMap = mvbMap[cameraId];
        }
        else if (mState == eTrackingState::LOST) {
            vCurrentKeys = mvCurrentKeys[cameraId];
        }
    } // destroy scoped mutex -> release mutex

    if (im.channels() < 3) // this should be always true
        cvtColor(im, im, cv::COLOR_GRAY2BGR);

    // Draw

    // Draw yolo rect
    for (size_t i = 0; i < vYoloBoundingBox.size(); i++) {
        cv::Rect2f tempRect = vYoloBoundingBox[i].GetRect();
        std::string tempLabel = vYoloBoundingBox[i].GetLabel();
        float confidence = vYoloBoundingBox[i].GetConfidence();

        const int left = tempRect.tl().x;
        const int top = tempRect.tl().y;

        char text[30];
        sprintf(text, "%s %4.2f", tempLabel.c_str(), confidence);

        assert(0 < col.count(tempLabel));

        constexpr int thickness = 2;
        const cv::Scalar color = col[tempLabel];
        cv::rectangle(im, tempRect.tl(), tempRect.br(), color, thickness);
        cv::rectangle(im, cv::Point(left, top - 20), cv::Point(left + 150, top), color, -1);
        cv::putText(im, text, tempRect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), thickness);
    }

    if (state == eTrackingState::NOT_INITIALIZED) // INITIALIZING
    {
        for (unsigned int i = 0; i < vMatches.size(); i++) {
            if (vMatches[i] >= 0 && vMatches[i] < vCurrentKeys.size()) {   // TODO: second condition is workaround
                cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
                         cv::Scalar(0, 255, 0));
            }
        }
    }
    else if (state == eTrackingState::OK) // TRACKING
    {
        if (drawPlaneSegment) {
            im = DrawPlaneSegmentResult(im, cameraId);
        }

        mnTracked = 0;
        mnTrackedVO = 0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for (int i = 0; i < n; i++) {
            if (vbVO[i] || vbMap[i]) {
                cv::Point2f pt1, pt2;
                pt1.x = vCurrentKeys[i].pt.x - r;
                pt1.y = vCurrentKeys[i].pt.y - r;
                pt2.x = vCurrentKeys[i].pt.x + r;
                pt2.y = vCurrentKeys[i].pt.y + r;

                // This is a match to a MapPoint in the map
                if (vbMap[i]) {
                    // cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    // cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im, state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawFrame(bool drawPlaneSegment, const double track_time) {
    cv::Mat img[4];
    for (int cameraId=0; cameraId < mnCameras; cameraId++) {
        img[cameraId] = DrawSingleFrame(drawPlaneSegment, cameraId);
    }

    std::ostringstream oss;
    double ms = track_time * 1e3;
    oss << std::fixed << std::setprecision(1) << ms;
    std::string text = "tracking time : " + oss.str() + "[msec]";

    cv::Mat dst;
    if (mnCameras == 1) {
        AddTextToBottom(img[0], text, 2.0, dst);
    } else if (mnCameras == 2) {
        cv::hconcat(img[0], img[1], img[2]);
        AddTextToBottom(img[2], text, 2.0, dst);
    } else if (mnCameras == 3) {
        int w = img[0].cols;
        int h = img[0].rows;
        img[3] = cv::Mat(h * 2, w * 2, img[0].type());
        img[0].copyTo(img[3].rowRange(0, h).colRange(0, w));
        img[1].copyTo(img[3].rowRange(0, h).colRange(w, w * 2));
        img[2].copyTo(img[3].rowRange(h, h * 2).colRange(0, w));
        AddTextToBottom(img[3], text, 2.0, dst);
    } else {  // mnCameras == 4
        cv::hconcat(img[0], img[1], dst);
        cv::hconcat(img[2], img[3], img[0]);
        cv::vconcat(dst, img[0], img[1]);
        AddTextToBottom(img[1], text, 2.0, dst);
    }

    return dst;
}

void FrameDrawer::DrawTextInfo(cv::Mat& im, int nState, cv::Mat& imText) {
    stringstream s;
    if (nState == eTrackingState::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if (nState == eTrackingState::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if (nState == eTrackingState::OK) {
        if (!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if (mnTrackedVO > 0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if (nState == eTrackingState::LOST) {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if (nState == eTrackingState::SYSTEM_NOT_READY) {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    AddTextToBottom(im, s.str(), 1.0, imText);
}

void FrameDrawer::AddTextToBottom(const cv::Mat& src, const std::string text, const double fontScale, cv::Mat& dst) {
    int baseline = 0;
    int thickness = 2;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_PLAIN, fontScale, thickness, &baseline);

    dst = cv::Mat(src.rows + textSize.height + 10, src.cols, src.type());
    src.copyTo(dst.rowRange(0, src.rows).colRange(0, src.cols));
    dst.rowRange(src.rows, dst.rows) = cv::Mat::zeros(textSize.height + 10, src.cols, src.type());
    cv::putText(dst, text, cv::Point(5, dst.rows - 5), cv::FONT_HERSHEY_PLAIN, fontScale, cv::Scalar(255, 255, 255), thickness, 8);
}

void FrameDrawer::Update(Tracking* pTracker) {
    unique_lock<mutex> lock(mMutex);
    int cameraId = pTracker->mCurrentFrame->mnCameraId;
    pTracker->mImGray.copyTo(mIm[cameraId]);
    mvCurrentKeys[cameraId] = pTracker->mCurrentFrame->mvKeys;
    yoloBoundingBoxList[cameraId] = pTracker->mCurrentFrame->GetYoloBoundingBoxList();
    mpPlaneSegmentImg[cameraId] = &(pTracker->mPlaneDetector.mSegmentImageOutput);
    N = mvCurrentKeys[cameraId].size();
    mvbVO[cameraId] = vector<bool>(N, false);
    mvbMap[cameraId] = vector<bool>(N, false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    if (pTracker->mLastProcessedState == eTrackingState::NOT_INITIALIZED) {
        mvIniKeys[cameraId] = pTracker->mInitialFrame[cameraId].mvKeys;
        mvIniMatches[cameraId] = pTracker->mvIniMatches[cameraId];
    }
    else if (pTracker->mLastProcessedState == eTrackingState::OK) {
        for (int i = 0; i < N; i++) {
            MapPoint* pMP = pTracker->mCurrentFrame->mvpMapPoints[i];
            if (pMP) {
                if (!pTracker->mCurrentFrame->mvbOutlier[i]) {
                    if (pMP->Observations() > 0)
                        mvbMap[cameraId][i] = true;
                    else
                        mvbVO[cameraId][i] = true;
                }
            }
        }
    }
    mState = static_cast<int>(pTracker->mLastProcessedState);
}

cv::Mat_<cv::Vec3b> FrameDrawer::DrawPlaneSegmentResult(cv::Mat rgbImg, int cameraId) {
    uchar* sCode;
    uchar* dColor;
    uchar* srgb;
    cv::Mat_<cv::Vec3b> seg_rz = cv::Mat_<cv::Vec3b>(rgbImg.rows, rgbImg.cols, cv::Vec3b(0, 0, 0));
    int code;
    for (int r = 0; r < rgbImg.rows; r++) {
        dColor = seg_rz.ptr<uchar>(r);
        sCode = mpPlaneSegmentImg[cameraId]->ptr<uchar>(r);
        srgb = rgbImg.ptr<uchar>(r);
        for (int c = 0; c < rgbImg.cols; c++) {
            code = *sCode;
            if (code > 0) {
                dColor[c * 3] = colorCodeForPlaneSegmentResult[code - 1][0] / 2 + srgb[0] / 2;
                dColor[c * 3 + 1] = colorCodeForPlaneSegmentResult[code - 1][1] / 2 + srgb[1] / 2;
                dColor[c * 3 + 2] = colorCodeForPlaneSegmentResult[code - 1][2] / 2 + srgb[2] / 2;
                ;
            }
            else {
                dColor[c * 3] = srgb[0];
                dColor[c * 3 + 1] = srgb[1];
                dColor[c * 3 + 2] = srgb[2];
            }
            sCode++;
            srgb++;
            srgb++;
            srgb++;
        }
    }

    return seg_rz;
}

} // namespace ORB_SLAM2
