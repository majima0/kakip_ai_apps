#include "calibrate.h"
#include <iostream>

void callback(const cv::Mat& img, void* arg) {
    cv::imshow("Calibration", img);
    cv::waitKey(100);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr 
            << "Usage: " << argv[0]
            << " path_to_camera_settings path_to_chessboard_settings [camera_index]" << std::endl;
        return 1;
    }

    std::vector<CameraInfo> cameras = CameraInfo::fromFile(argv[1]);
    if(cameras.empty()) {
        std::cerr << "Failed to open settings file at: " << argv[1] << std::endl;
        return -1;
    }

    ChessboardInfo chessboard = ChessboardInfo::fromFile(argv[2]);
    if(chessboard.rows == 0) {
        std::cerr << "Failed to open settings file at: " << argv[2] << std::endl;
        return -1;
    }

    CameraInfo camera;
    if(cameras.size() == 1) {
        camera = cameras[0];
    } else {
        if(argc != 4) {
            std::cerr << "Camera setting file has multiple cameras. Second argument 'camera_index' must be set." << std::endl;
            return 1;
        }
        camera = cameras[std::atoi(argv[3])];
    }

    float err = calibrateSingle(camera, chessboard, 20, 0, callback);
    if(err < 0.) {
        return -1;
    }

    std::cout << "error = " << err << std::endl; 
    std::cout << std::endl;

    std::cout << "    fx: " << camera.fx << std::endl
              << "    fy: " << camera.fy << std::endl
              << "    cx: " << camera.cx << std::endl
              << "    cy: " << camera.cy << std::endl
              << "    k1: " << camera.k1 << std::endl
              << "    k2: " << camera.k2 << std::endl
              << "    p1: " << camera.p1 << std::endl
              << "    p2: " << camera.p2 << std::endl;
    if(camera.k3 != 0.)
        std::cout << "    k3: " << camera.k3 << std::endl;
    return 0;
}