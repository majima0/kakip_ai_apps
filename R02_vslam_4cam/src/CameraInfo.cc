
#include <CameraInfo.h>

CameraInfo::CameraInfo(const cv::FileNode& camera) {
    width = camera["width"];
    height = camera["height"];
    fx = camera["fx"];
    fy = camera["fy"];
    cx = camera["cx"];
    cy = camera["cy"];
    k1 = camera["k1"];
    k2 = camera["k2"];
    p1 = camera["p1"];
    p2 = camera["p2"];
    k3 = camera["k3"];
    device = (std::string)camera["device"];
    fps = camera["fps"];
    bf = camera["bf"];
    pose = camera["pose"].mat();
    if (!camera["wait_ms"].isNone())
        wait_ms = camera["wait_ms"];
}

std::vector<CameraInfo> CameraInfo::fromFile(const char* filename) {
    std::vector<CameraInfo> info;

    cv::FileStorage fsSettings(filename, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
        return info;

    cv::FileNode root = fsSettings.root();
    std::vector<std::string> keys = root.keys();
    if(std::find(keys.begin(), keys.end(), "Cameras") == keys.end()) {
        CameraInfo elem;
        elem.width = root["Camera.width"];
        elem.height = root["Camera.height"];
        elem.fx = root["Camera.fx"];
        elem.fy = root["Camera.fy"];
        elem.cx = root["Camera.cx"];
        elem.cy = root["Camera.cy"];
        elem.k1 = root["Camera.k1"];
        elem.k2 = root["Camera.k2"];
        elem.p1 = root["Camera.p1"];
        elem.p2 = root["Camera.p2"];
        elem.k3 = root["Camera.k3"];
        elem.device = "/dev/video0";
        elem.fps = root["Camera.fps"];
        elem.bf = root["Camera.bf"];
        if (!root["Camera.wait_ms"].isNone())
            elem.wait_ms = root["Camera.wait_ms"];
        info.push_back(elem);
    } else {
        cv::FileNode seq = root["Cameras"];
        for(auto node : seq) {
            CameraInfo elem(node);
            info.push_back(elem);
        }
    }
    return info;
}

void CameraInfo::save(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::Mode::WRITE);
    fs.write("width", width);
    fs.write("height", height);
    fs.write("fx", fx);
    fs.write("fy", fy);
    fs.write("cx", cx);
    fs.write("cy", cy);
    fs.write("k1", k1);
    fs.write("k2", k2);
    fs.write("p1", p1);
    fs.write("p2", p2);
    fs.write("k3", k3);
    fs.write("device", device);
    fs.write("fps", fps);
    fs.write("bf", bf);
    fs.write("pose", pose);
    fs.write("wait_ms", wait_ms);
}