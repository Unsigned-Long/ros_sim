#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include "artwork/logger/logger.h"
#include "pcl-viewer/scene_viewer.h"

std::vector<std::string> splitString(const std::string &str, char splitor, bool ignoreEmpty = true) {
    std::vector<std::string> vec;
    auto iter = str.cbegin();
    while (true) {
        auto pos = std::find(iter, str.cend(), splitor);
        auto elem = std::string(iter, pos);
        if (!(elem.empty() && ignoreEmpty)) {
            vec.push_back(elem);
        }
        if (pos == str.cend()) {
            break;
        }
        iter = ++pos;
    }
    return vec;
}

std::string readString(std::fstream &file) {
    if (!file.is_open())
        throw std::runtime_error("file open failed in 'std::string readString(std::fstream &file)'");
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::string str(size, ' ');
    file.read(const_cast<char *>(str.c_str()), size);
    return str;
}

void SavePoseToSDF(const std::vector<ns_viewer::Posed> &poseSeq, const std::string &filename) {
    std::ofstream file(filename, std::ios::out);
    file << "<script>" << std::endl;
    file << "<loop>1</loop>" << std::endl;
    file << "<auto_start>1</auto_start>" << std::endl;
    file << "<trajectory id='0' type='square'>" << std::endl;
    for (const auto &pose: poseSeq) {
        file << "<waypoint>" << std::endl;
        file << "<time>" << pose.timeStamp << "</time>" << std::endl;
        file << "<pose>"
             << pose.translation(0) << ' '
             << pose.translation(1) << ' '
             << pose.translation(2) << ' '
             << 0.0 << ' ' << 0.0 << ' ' << 0.0
             << "</pose>" << std::endl;
        file << "</waypoint>" << std::endl;
    }
    file << "</trajectory>" << std::endl;
    file << "</script>" << std::endl;

    file.close();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gen_traj");
    std::fstream file("/home/csl/ros_ws/sim-scene/src/model/trajectory/trajectory.txt", std::ios::in);
    auto str = readString(file);
    file.close();

    auto sVec = splitString(str, ' ');
    std::vector<double> dVec(sVec.size());
    for (int i = 0; i < sVec.size(); ++i) {
        dVec.at(i) = std::stod(sVec.at(i));
    }

    const int POSE_SIZE = dVec.size() / 16;
    const double SCALE_FACTOR = 0.15;

    ns_viewer::SceneViewer viewer;
    std::vector<ns_viewer::Posed> pose(POSE_SIZE);
    for (int i = 0; i < POSE_SIZE; ++i) {
        Eigen::Matrix4d mat;
        mat << dVec.at(i * 16 + 0), dVec.at(i * 16 + 1), dVec.at(i * 16 + 2), dVec.at(i * 16 + 3),
                dVec.at(i * 16 + 4), dVec.at(i * 16 + 5), dVec.at(i * 16 + 6), dVec.at(i * 16 + 7),
                dVec.at(i * 16 + 8), dVec.at(i * 16 + 9), dVec.at(i * 16 + 10), dVec.at(i * 16 + 11),
                dVec.at(i * 16 + 12), dVec.at(i * 16 + 13), dVec.at(i * 16 + 14), dVec.at(i * 16 + 15);
        mat.topLeftCorner<3, 3>() /= SCALE_FACTOR;
        pose.at(i) = ns_viewer::Posed(mat, i / 20.0);
        viewer.AddPose(pose.at(i).cast<float>());
        LOG_VAR(i, mat);
    }
    viewer.RunSingleThread();
    SavePoseToSDF(pose, "/home/csl/ros_ws/sim-scene/src/model/trajectory/trajectory.xml");
    ros::shutdown();
    return 0;
}
