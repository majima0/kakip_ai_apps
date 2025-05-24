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
#include "YoloDetector.h"

struct Message {
    std::string state;
    std::vector<cv::Mat> points;
    cv::Mat pose;
    std::vector<cv::Mat> images;
    std::vector<ORB_SLAM2::YoloBoundingBox> boundingBoxes;

    void dump(std::ostream& os) {
		os << "{";
        os << "\"state\": \"" << state << "\", ";
        os << "\"points\": [";
        os << std::setprecision(3);
        for (int i=0; i<points.size(); ++i) {
            const cv::Mat& p = points[i];
            os << p.at<float>(0) << ", "
               << p.at<float>(1) << ", "
               << p.at<float>(2);
            if (i < points.size() - 1)
                os << ", ";
        }
        os << "]";
        if (!pose.empty()) {
            os << ", ";

            cv::Mat Twc = pose.inv();
            // 0: initial orientation, +: left, -: right
            float yaw = std::atan2(-Twc.at<float>(0, 2), Twc.at<float>(2, 2));

            os << "\"pose\": [";
            os << std::setprecision(5);
            os << Twc.at<float>(0, 3) << ", "
               << Twc.at<float>(1, 3) << ", "
               << Twc.at<float>(2, 3) << ", "
               << yaw << "]";

            os << ", ";
            os << "\"objects\": [";
            os << std::setprecision(3);
            bool firstObject = true;
            for(ORB_SLAM2::YoloBoundingBox bb : boundingBoxes) {
                if (!firstObject) os << ", ";
                cv::Rect2f rect = bb.GetRect();
                os << "{";
                os << "\"class\": \"" << bb.GetLabel() << "\", ";
                os << "\"score\": " << bb.GetConfidence() << ", ";
                os << "\"top\": " << rect.tl().y << ", ";
                os << "\"left\": " << rect.tl().x << ", ";
                os << "\"bottom\": " << rect.br().y << ", ";
                os << "\"right\": " << rect.br().x;
                os << "}";
                firstObject = false;
            }
            os << "]";
        }

        int cameraId = 0;
        for (cv::Mat img : images) {
            ++cameraId;
            if (img.empty())
                continue;
            std::vector<unsigned char> buf;
            bool ok = cv::imencode(".jpg", img, buf);
            if (!ok)
                continue;
            std::string encoded = clx::base64::encode((char*)buf.data(), buf.size());
            os << ", ";
            os << "\"image" << cameraId << "\": \"" << encoded << "\"";
        }
        os << "}" << std::endl;        
    }

    void dump_rosbridge(std::ostream& os) {
        // /state
		os << "{";
        os << "\"op\": \"publish\", ";
        os << "\"topic\": \"/state\", ";
        os << "\"msg\": {\"data\": \"" << state << "\"}";
		os << "}" << std::endl;

        // /points
		os << "{";
        os << "\"op\": \"publish\", ";
        os << "\"topic\": \"/points\", ";
        os << "\"msg\": {\"data\": [";
        os << std::setprecision(3);
        for (int i=0; i<points.size(); ++i) {
            const cv::Mat& p = points[i];
            os << p.at<float>(0) << ", "
               << p.at<float>(1) << ", "
               << p.at<float>(2);
            if (i < points.size() - 1)
                os << ", ";
        }
        os << "]}"; 
		os << "}" << std::endl;

        if (!pose.empty()) {
            cv::Mat Twc = pose.inv();
            // 0: initial orientation, +: left, -: right
            float yaw = std::atan2(-Twc.at<float>(0, 2), Twc.at<float>(2, 2));

            // /raw_pose
    		os << "{";
            os << "\"op\": \"publish\", ";
            os << "\"topic\": \"/raw_pose\", ";
            os << "\"msg\": {\"data\": [";
            os << std::setprecision(5);
            os << Twc.at<float>(0, 3) << ", "
               << Twc.at<float>(1, 3) << ", "
               << Twc.at<float>(2, 3) << ", "
               << yaw << "]}";
		    os << "}" << std::endl;

            // /objects
    		os << "{";
            os << "\"op\": \"publish\", ";
            os << "\"topic\": \"/objects\", ";
            os << "\"msg\": {\"camera_pose\": [";
            os << Twc.at<float>(0, 3) << ", "
               << Twc.at<float>(1, 3) << ", "
               << Twc.at<float>(2, 3) << ", "
               << yaw << "], ";
            os << "\"objects\": [";
            os << std::setprecision(3);
            bool firstObject = true;
            for(ORB_SLAM2::YoloBoundingBox bb : boundingBoxes) {
                if (!firstObject) os << ", ";
                cv::Rect2f rect = bb.GetRect();
                os << "{";
                os << "\"class\": \"" << bb.GetLabel() << "\", ";
                os << "\"score\": " << bb.GetConfidence() << ", ";
                os << "\"top\": " << rect.tl().y << ", ";
                os << "\"left\": " << rect.tl().x << ", ";
                os << "\"bottom\": " << rect.br().y << ", ";
                os << "\"right\": " << rect.br().x;
                os << "}";
                firstObject = false;
            }
            os << "]}";
		    os << "}" << std::endl;
        }

        int cameraId = 0;
        for (cv::Mat img : images) {
            ++cameraId;
            if (img.empty())
                continue;
            std::vector<unsigned char> buf;
            bool ok = cv::imencode(".jpg", img, buf);
            if (!ok)
                continue;
            std::string encoded = clx::base64::encode((char*)buf.data(), buf.size());

            // /image{cameraId}_bin
    		os << "{";
            os << "\"op\": \"publish\", ";
            os << "\"topic\": \"/image" << cameraId << "_bin\", ";
            os << "\"msg\": {\"data\": \"" << encoded << "\"}";
		    os << "}" << std::endl;
        }
    }

    static Message calibration(int nCameras, int cameraId, const cv::Mat& image) {
        Message msg;
        msg.state = "CALIBRATION";
        msg.images.resize(nCameras);
        msg.images[cameraId] = image.clone();
        return msg;
    }

    static Message noImagesYet() {
        Message msg;
        msg.state = "NO_IMAGES_YET";
        return msg;
    }

    static Message notInitialized() {
        Message msg;
        msg.state = "NOT_INITIALIZED";
        return msg;
    }

    static Message ok(
        const std::vector<ORB_SLAM2::MapPoint*>& mps,
        const cv::Mat& pose,
        const std::vector<ORB_SLAM2::YoloBoundingBox>& boundingBoxes) {
        Message msg = Message::lost(mps);
        msg.state = "OK";
        msg.pose = pose.clone();
        msg.boundingBoxes = boundingBoxes;
        // for test
        //msg.boundingBoxes.push_back(ORB_SLAM2::YoloBoundingBox(1., 2., 3., 4., "test", 0.9));
        return msg;
    }

    static Message lost(const std::vector<ORB_SLAM2::MapPoint*>& mps) {
        Message msg;
        msg.state = "LOST";
        msg.points.reserve(mps.size());
        for (auto mp : mps) {
            if (mp->isBad())
                continue;
            msg.points.push_back(mp->GetWorldPos());
        }
        return msg;
    }
};

class Client {
public:
    Client(std::string strServer, int intPort) : server(strServer), port(intPort), terminating(false), thread(&Client::run, this)
    {
        if (Client::instance != nullptr) {
            std::cerr << "Do not create multiple instances of class Client at once." << std::endl;
            std::exit(1);
        }
        Client::instance = this;

        struct sigaction action;
        sigset_t sigset;
        sigemptyset(&sigset);
        action.sa_handler = &Client::sigpipe_handler;
        action.sa_flags = 0;
        action.sa_mask = sigset;
        sigaction(SIGPIPE, &action, NULL);
    }

    Client(const cv::FileNode& node) : Client(node["address"], node["port"])
    {}
    ~Client() {
        terminating = true;
        thread.join();

        Client::instance = nullptr;
    }

    static void sigpipe_handler(int signal) {
        Client::instance->socket.close();
    }

    clx::tcp::sockstream getStream() {
        return clx::tcp::sockstream(socket);
    }

    bool isConnected() {
        return socket.is_connect();
    }

    void run() {
        while(!terminating) {
            try {
		        socket.connect(server, port);
            } catch(clx::socket_error& ex) {
                std::cerr << "Trying to connect to Raspberry Pi... Exception: " << ex.what() << std::endl;
                socket.close();
            } catch(clx::sockaddress_error& ex) {
                std::cerr << "Trying to connect to Raspberry Pi... Exception: " << ex.what() << std::endl;
                socket.close();
            }
            usleep(1000000);
        }
        socket.close();
    }

protected:
    std::string server;
    int port;
    bool terminating;
    clx::tcp::socket socket;
    std::thread thread;

    static Client* instance;
};

Client* Client::instance = nullptr;

struct CalibrationContext {
    CalibrationContext(int nCam, Client* cli) {
        nCameras = nCam;
        client = cli;
        cameraId = 0;
    }

    Client* client;
    int cameraId;
    int nCameras;
};

void sendCalibrationImage(const cv::Mat& img, void* obj) {
    try {
        CalibrationContext* ctx = (CalibrationContext*)obj;
        clx::tcp::sockstream ss = ctx->client->getStream();
        Message msg = Message::calibration(ctx->nCameras, ctx->cameraId, img);
        //msg.dump(ss);
        msg.dump_rosbridge(ss);
    } catch(std::exception& ex) {
        std::cerr << "Failed to send an image to Raspberry Pi. Exception: " << ex.what() << std::endl;
    }
}

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

    cv::FileNode srv = fsSettings["Server"];
    float yMin = -std::numeric_limits<float>::infinity();
    float yMax = std::numeric_limits<float>::infinity();
    if (!srv["y_min"].isNone())
        yMin = srv["y_min"];
    if (!srv["y_min"].isNone())
        yMax = srv["y_max"];
    std::cout << "Condition for map points to be published: " << yMin << " < y < " << yMax << std::endl;

    Client client(srv);

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
    std::cout << "Following cameras are not yet calibrated: [";
    for(int i : cameraIdsToCalibrate) {
        std::cout << i << ", ";
    } 
    std::cout << "]." << std::endl;

    ChessboardInfo chessboard(fsSettings["Chessboard"]);
    std::cout << "Chessboard.rows: " << chessboard.rows << std::endl;
    std::cout << "Chessboard.cols: " << chessboard.cols << std::endl;
    std::cout << "Chessboard.cellSize: " << chessboard.cellSize << std::endl;

    unsigned int calibNumSamples = 20;
    if (!fsSettings["Calibration"]["numSamples"].isNone())
        calibNumSamples = (int)(fsSettings["Calibration"]["numSamples"]); 
    unsigned int calibFrameSkips = 0;
    if (!fsSettings["Calibration"]["frameSkips"].isNone())
        calibFrameSkips = (int)(fsSettings["Calibration"]["frameSkips"]); 

    bool fixPrincipalPoint, fixAspectRatio, zeroTangentDist, fixK1, fixK2, fixK3;
    fixPrincipalPoint = (std::string)fsSettings["Calibration"]["fixPrincipalPoint"] == "true";
    fixAspectRatio = (std::string)fsSettings["Calibration"]["fixAspectRatio"] == "true";
    zeroTangentDist = (std::string)fsSettings["Calibration"]["zeroTangentDist"] == "true";
    fixK1 = (std::string)fsSettings["Calibration"]["fixK1"] == "true";
    fixK2 = (std::string)fsSettings["Calibration"]["fixK2"] == "true";
    fixK3 = (std::string)fsSettings["Calibration"]["fixK3"] == "true";

    CalibrationContext ctx(cameras.size(), &client);
    while (cameraIdsToCalibrate.size() > 0) {
        auto it=cameraIdsToCalibrate.begin();
        while(it != cameraIdsToCalibrate.end()) {
            ctx.cameraId = *it;
            std::cout << "Calibrating cameara[" << ctx.cameraId << "a]" << std::endl;
            float err = calibrateSingle(cameras[ctx.cameraId], chessboard, calibNumSamples, calibFrameSkips,
                                        fixPrincipalPoint, fixAspectRatio, zeroTangentDist, fixK1, fixK2, fixK3,
                                        sendCalibrationImage, &ctx);
            if (err < 0.) {
                std::cout << "Failed to calibrate camera[" << ctx.cameraId << "]." << std::endl;
                ++it;
            } else {
                std::string filename = getCameraSettingFileName(ctx.cameraId);
                cameras[ctx.cameraId].save(filename);

                std::cout << "Successfully calibrated camera[" << ctx.cameraId << "] with error=" << err << "." << std::endl;
                std::cout << "Saved to " << filename << "." << std::endl;

                it = cameraIdsToCalibrate.erase(it);
            }
        }
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

        if (client.isConnected()) {
            mapPoints = SLAM.GetAllMapPoints();
            mapPointsFiltered.clear();
            for (auto mp : mapPoints) {
                if (mp->isBad())
                    continue;
                float y = mp->GetWorldPos().at<float>(1);
                if (yMin < y && y < yMax)
                    mapPointsFiltered.push_back(mp);
            }

            try {
                clx::tcp::sockstream ss = client.getStream();
                Message msg;
                switch(SLAM.GetTrackingState()) {
                case eTrackingState::SYSTEM_NOT_READY:
                case eTrackingState::NO_IMAGES_YET:
                    msg = Message::noImagesYet();
                    break;
                case eTrackingState::NOT_INITIALIZED:
                    msg = Message::notInitialized();
                    break;
                case eTrackingState::OK:
                    msg = Message::ok(
                        mapPointsFiltered, Tcw, SLAM.mpTracker->mCurrentFrame->GetYoloBoundingBoxList());
                    break;
                case eTrackingState::LOST:
                    msg = Message::lost(mapPointsFiltered);
                    break;
                }
                msg.dump_rosbridge(ss);
            } catch(std::exception& ex) {
                std::cerr << "Failed to send a message to Raspberry Pi. Exception: " << ex.what() << std::endl;
            }
        }

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
