#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include "artwork/logger/logger.h"
#include "tinyxml2.h"
#include "artwork/csv/csv.h"

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

std::vector<std::array<double, 7>> ReadPoseVecFromDEA(const std::string &str) {
    LOG_PROCESS("start reading trajectory dea file: ", str)

    tinyxml2::XMLDocument doc;
    doc.LoadFile(str.c_str());
    auto animation = doc.FirstChildElement()->FirstChildElement("library_animations")->FirstChildElement();

    auto output = animation->FirstChildElement()->FirstChildElement()->NextSiblingElement();
    auto outputAry = output->FirstChildElement()->GetText();

    auto elems = splitString(std::string(outputAry), ' ');

    LOG_INFO("the elem count is: ", elems.size(), ", pose count is: ", elems.size() / 16);

    std::vector<std::array<double, 7>> poseAry(elems.size() / 16);

    const double scale = 1.0 / 0.15;
    for (int i = 0; i < poseAry.size(); ++i) {
        Eigen::Matrix4d pose;
        for (int j = 0; j < 16; ++j) {
            pose(j / 4, j % 4) = std::stod(elems[i * 16 + j]);
        }

        Eigen::Vector3d t(pose.topRightCorner<3, 1>());
        Eigen::Quaterniond q(pose.topLeftCorner<3, 3>() * scale);
        auto &elem = poseAry.at(i);
        // x y z qx qy qz qw
        elem.at(0) = t(0);
        elem.at(1) = t(1);
        elem.at(2) = t(2);
        elem.at(3) = q.x();
        elem.at(4) = q.y();
        elem.at(5) = q.z();
        elem.at(6) = q.w();
    }
    LOG_INFO("organize pose sequence finished...");

    return poseAry;
}

void SavePoseSequence(const std::string &str, const std::vector<std::array<double, 7>> &poseSeq) {
    auto writer = ns_csv::CSVWriter::create(str);

    for (const auto &item: poseSeq) {
        writer->writeLine(',', item[0], item[1], item[2], item[3], item[4], item[5], item[6]);
    }

    LOG_PROCESS("save trajectory dea to file: ", str)
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "generate_actor");

    auto poseSeq = ReadPoseVecFromDEA(
            "/home/csl/ros_ws/sim-scene/src/trajectory_controller/trajectory/trajectory.dae"
    );

    SavePoseSequence(
            "/home/csl/ros_ws/sim-scene/src/trajectory_controller/trajectory/trajectory.csv",
            poseSeq
    );

    ros::shutdown();
    return 0;
}
