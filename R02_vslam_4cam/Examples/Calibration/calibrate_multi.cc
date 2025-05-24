#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream> 
#include <chrono>

#include <errno.h>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

#include "System.h"
#include "Converter.h"
#include "Enum.h"

#include "measure_time.h"

#if defined(ENABLE_DRP)
#include "drp.h"
#endif

using namespace std;

class CalibrationSystem : public ORB_SLAM2::System {
public:
    CalibrationSystem(
        const string& strVocFile, const string& strSettingsFile,
        const bool bUsePangolinViewer = false, const bool bUseSocketViewer = true,
        const unsigned int drp_dev_num = 0, const unsigned int drp_ai_dev_num = 0,
        unsigned long nKeyFramesToStart=50);
    
    cv::Mat TrackMonocular();

    std::vector<ORB_SLAM2::Tracking*> mvpSlaveTrackers;
    std::vector<ORB_SLAM2::ImageProcessing*> mvpSlaveImageProcessors;
    std::vector<std::vector<pair<double, cv::Mat>>> slaveTrajectories;
    unsigned long mnKeyFramesToStart;
    bool mbStarted;
};

class YoloDetector_test : public ORB_SLAM2::YoloDetector_drp {
public:
    YoloDetector_test() {}

    bool YoloObjectDetect(const cv::Mat img, std::vector<ORB_SLAM2::YoloBoundingBox>& yoloBoundingBoxList) {
        cout << "YoloObjectDetect" << endl;
        return YoloDetector_test::YoloObjectDetect(img, yoloBoundingBoxList);
    }
};

CalibrationSystem::CalibrationSystem(const string& strVocFile, const string& strSettingsFile,
               const bool bUsePangolinViewer, const bool bUseSocketViewer,
               const unsigned int drp_dev_num, const unsigned int drp_ai_dev_num,
               unsigned long nKeyFramesToStart)
: ORB_SLAM2::System(strVocFile, strSettingsFile,
                    InputType::MULTI_CAMERA_CALIBRATION, eSensor::MONOCULAR,
                    bUsePangolinViewer, bUseSocketViewer, drp_dev_num, drp_ai_dev_num),
  mnKeyFramesToStart(nKeyFramesToStart), mbStarted(false) {
    int nCams = mpImageLoaders.size();
    slaveTrajectories.resize(nCams-1);
}

cv::Mat CalibrationSystem::TrackMonocular() {
    MT_START(mt_tracker_wait_for_next_frame);
    while (mpTracker->wait_for_next_frame.load()) {
        usleep(1000); // Sleep 1ms
    }
    MT_FINISH(mt_tracker_wait_for_next_frame);

    if (mbStarted)
        mpTracker->mState = eTrackingState::LOST;

    MT_START(mt_grab);
    cv::Mat Tcw = mpTracker->GrabImageMonocular();
    MT_FINISH(mt_grab);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;

    //trajectory.push_back({mpTracker->timestamp, Tcw});
    int cameraId = mpTracker->mCurrentFrame->mnCameraId;
    if (cameraId == 0) {
        trajectory.push_back({mpTracker->timestamp, Tcw});
    } else {
        slaveTrajectories[cameraId-1].push_back({mpTracker->timestamp, Tcw});
    }

    {
        MT_START(mt_wait_for_next_max_features_to_true);
        while (!mpImageProcessor->wait_for_next_max_features.load()) {
            usleep(1000); // Sleep 1ms
        }
        MT_FINISH(mt_wait_for_next_max_features_to_true);

        if (mTrackingState == eTrackingState::NOT_INITIALIZED
            || mTrackingState == eTrackingState::NO_IMAGES_YET)
            mpImageProcessor->SetMaxFeatures(2 * mpTracker->nFeatures);
        else
            mpImageProcessor->SetMaxFeatures(mpTracker->nFeatures);
    }

    mpTracker->wait_for_next_frame.store(true);

    if (!mbStarted) {
        if (mTrackingState == eTrackingState::OK && mpMap->KeyFramesInMap() > mnKeyFramesToStart) {
            cout << "Starting calibration." << endl;
            int nCam = mpImageLoaders.size();
            mpTracker->InformOnlyTracking(true);
            for(int i=1; i<nCam; ++i) {
                mpImageLoaders[i]->SeImageProcessor(mpImageProcessor);
            }
            mbStarted = true;
        }
    }

    return Tcw;
}

bool interrupt = false;
void signal_handler(const int in) {
    if (in == SIGINT)
        interrupt = true;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << endl
             << "Usage: ./calibrate_multi path_to_vocabulary path_to_settings [num_key_frames_to_start]" << endl;
        return 1;
    }
    unsigned long nKeyFramesToStart = 50;
    if (argc >= 4) {
        nKeyFramesToStart = std::atol(argv[3]);
    }

    // Check settings file
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << argv[2] << endl;
        exit(-1);
    }

    const int sleep_ms = (fsSettings["Sleep"].empty()) ? 0 : fsSettings["Sleep"];
    if (0 < sleep_ms)
        std::cout << "Sleep " << sleep_ms << "ms every frame." << std::endl;

    MT_INIT();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const bool bUsePangolinViewer = (((std::string)fsSettings["Viewer.Type"]) == "PangolinViewer");
    const bool bUseSocketViewer = (((std::string)fsSettings["Viewer.Type"]) == "SocketViewer");
    const unsigned int drp_dev_num = 1;
    const unsigned int drp_ai_dev_num = 0;
    CalibrationSystem SLAM(argv[1], argv[2], bUsePangolinViewer, bUseSocketViewer, drp_dev_num, drp_ai_dev_num, nKeyFramesToStart);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    printf("Press Ctrl+C to exit.\n");
    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        fprintf(stderr, "Failed to setup signal_handler.\n");
        return 1;
    }

    // Main loop
    MT_START(mt_main_loop);
    int nImages = 0;
    while (!interrupt) {
        // #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // #else
        //         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        // #endif

        // Pass the image to the SLAM system
        MT_START(mt_main_track);
        SLAM.TrackMonocular();
        MT_FINISH(mt_main_track);

        MT_START(mt_option_usleep);
        if (0 < sleep_ms)
            usleep(sleep_ms * 1e3);
        MT_FINISH(mt_option_usleep);

        // #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // #else
        //         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        // #endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        SLAM.SetTrackTime(ttrack);

        vTimesTrack.push_back(ttrack);

        nImages++;
    }

    MT_FINISH(mt_main_loop);

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    cout << "camera_0:" << SLAM.trajectory.size() << endl;
    int i = 1;
    for (auto traj : SLAM.slaveTrajectories) {
        ofstream ofs;
        stringstream ss;
        ss << "camera_0_" << i << "_trajectory.txt";
        ofs.open(ss.str().c_str());
        
        cout << "camera_" << i << ": " << traj.size() << endl;

        for (auto p : traj) {
            long double t = p.first;
            cv::Mat Tcw = p.second;
            if (Tcw.rows != 4 || Tcw.cols != 4)
                continue;
            
            long double t0, t1, dt;
            cv::Mat Tcw0, Tcw1;
            t1 = SLAM.trajectory.front().first;
            Tcw1 = SLAM.trajectory.front().second;
            for (auto q : SLAM.trajectory) {
                if (q.second.rows != 4 || q.second.cols != 4)
                    continue;
                if (t1 < t || Tcw0.rows != 4 || Tcw0.cols != 4) {
                    t0 = t1;
                    Tcw0 = Tcw1;
                }
                t1 = q.first;
                Tcw1 = q.second;
                if (t1 >= t && Tcw1.rows == 4 && Tcw1.cols == 4)
                    break;
            }
            cout << setprecision(15) << t0 << " <- " << t << " -> " << t1 << endl;
            if (!(t0 <= t && t <= t1))
                continue;
            if (Tcw0.rows != 4 || Tcw0.cols != 4)
                continue;
            if (Tcw1.rows != 4 || Tcw1.cols != 4)
                continue;

            //dt = t1 - t0;
            //if (dt > 0.02)
            //    continue;

            Eigen::Matrix<double, 3, 3> TcwEig, Tcw0Eig, Tcw1Eig;
            TcwEig = ORB_SLAM2::Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
            Tcw0Eig = ORB_SLAM2::Converter::toMatrix3d(Tcw0.rowRange(0,3).colRange(0,3));
            Tcw1Eig = ORB_SLAM2::Converter::toMatrix3d(Tcw1.rowRange(0,3).colRange(0,3));
            Eigen::Quaterniond qs(TcwEig), q0(Tcw0Eig), q1(Tcw1Eig);

            double alpha = (double)((t - t0) / (t1 - t0));
            Eigen::Quaterniond qm = q0.slerp(alpha, q1);
            cout << t0 << " " << t1 << " " << t1 - t0 << " " << alpha << ", " << Tcw0 << ", " << Tcw1 << endl;

            double ts0 = Tcw.at<float>(0, 3); 
            double ts1 = Tcw.at<float>(1, 3); 
            double ts2 = Tcw.at<float>(2, 3); 
            double tm0 = (1 - alpha) * Tcw0.at<float>(0, 3) + alpha * Tcw1.at<float>(0, 3); 
            double tm1 = (1 - alpha) * Tcw0.at<float>(1, 3) + alpha * Tcw1.at<float>(1, 3); 
            double tm2 = (1 - alpha) * Tcw0.at<float>(2, 3) + alpha * Tcw1.at<float>(2, 3); 
            
            ofs << setprecision(15) << t0 << " " << t << " " << t1 << " " << setprecision(9)
                << tm0 << " " << tm1 << " " << tm2 << " "
                << qm.x() << " " << qm.y() << " " << qm.z() << " " << qm.w() << " "
                << ts0 << " " << ts1 << " " << ts2 << " "
                << qs.x() << " " << qs.y() << " " << qs.z() << " " << qs.w() << endl;
        }
        ++i;
    }

    MT_PRINT();

    return 0;
}