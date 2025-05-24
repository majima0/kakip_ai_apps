#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <clx/tcp.h>
#include <clx/base64.h>
#include <signal.h>
#include <limits>

#include <opencv2/opencv.hpp>

#include <System.h>
#include <MapPoint.h>
#include "calibrate.h"

const std::string SETTING_FILE("Demo/settings.yaml");
const std::string VOCABULARY_FILE("Vocabulary/ORBvoc.txt");
const std::string CAMERA_SETTING_FILE_PREFIX("Demo/camera");
const std::string CAMERA_SETTING_FILE_SUFFIX("_settings.yaml");

int main(int argc, char* argv[]) {
    cv::FileStorage fsSettings(SETTING_FILE, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << SETTING_FILE << std::endl;
        return -1;
    }

    int wait_ms = 0;
    if (!fsSettings["wait_ms"].isNone())
        wait_ms = fsSettings["wait_ms"];

    std::vector<CameraInfo> cameras;
    for (CameraInfo cam : fsSettings["Cameras"])
        cameras.push_back(CameraInfo(cam));

    if (cameras.size() < 1) {
        std::cerr << "No camera settings found." << std::endl;
        return -1;
    } else {
        std::cout << "Found " << cameras.size() << " camera settings." << std::endl;
    }

    auto getCameraSettingFileName = [&] (int i) {
        std::stringstream ss;
        ss << CAMERA_SETTING_FILE_PREFIX << (i + 1) << CAMERA_SETTING_FILE_SUFFIX;
        return ss.str();
    };

    std::vector<int> cameraIdsToCalibrate;
    for (int i=0; i<cameras.size(); ++i) {
        if (cameras[i].fx == 0.) {
            std::string filename = getCameraSettingFileName(i);
            cv::FileStorage setting(filename, cv::FileStorage::READ);
            if (!setting.isOpened()) {
                cameraIdsToCalibrate.push_back(i);
            } else {
                std::cout << "Loading camera parameters from " << filename << "." << std::endl;
                CameraInfo& cam = cameras[i];
                cam.fx = setting["fx"];
                cam.fy = setting["fy"];
                cam.cx = setting["cx"];
                cam.cy = setting["cy"];
                cam.k1 = setting["k1"];
                cam.k2 = setting["k2"];
                cam.p1 = setting["p1"];
                cam.p2 = setting["p2"];
                if (!setting["k3"].isNone())
                    cam.k3 = setting["k3"];
            }
        }
    }
    if(cameraIdsToCalibrate.size() > 0) {
        std::cout << "Following cameras are not yet calibrated: [";
        for(int i : cameraIdsToCalibrate) {
            std::cout << i << ", ";
        } 
        std::cout << "]." << std::endl;
        return 1;
    }

    std::cout << "Starting visual SLAM." << std::endl;
	
    const bool useSocketViewer = (((std::string)fsSettings["Viewer.Type"]) == "SocketViewer");
    const unsigned int drp_dev_num = 1;
    const unsigned int drp_ai_dev_num = 0;
    ORB_SLAM2::System SLAM(VOCABULARY_FILE, SETTING_FILE, InputType::MULTI_CAMERA, eSensor::MONOCULAR, false,
                           useSocketViewer, drp_dev_num, drp_ai_dev_num, cameras);

    int nImages = 0;
    std::vector<ORB_SLAM2::MapPoint*> mapPoints, mapPointsFiltered;
    while (true) {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cv::Mat Tcw = SLAM.TrackMonocular();

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        SLAM.SetTrackTime(ttrack);

        nImages++;
        usleep(wait_ms * 1000);
    }

    // Stop all threads
    SLAM.Shutdown();

	return 0;
}
