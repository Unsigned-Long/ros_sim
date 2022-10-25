#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include "artwork/logger/logger.h"
#include "pcl-viewer/scene_viewer.h"
#include "tinyxml2.h"

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

std::vector<std::array<double, 6>> ReadPoseVecFromDEA(const std::string &str) {
    tinyxml2::XMLDocument doc;
    doc.LoadFile(str.c_str());
    auto animation = doc.FirstChildElement()->FirstChildElement("library_animations")->FirstChildElement();
    auto robot_location_X = animation->FirstChildElement();
    auto robot_location_Y = robot_location_X->NextSiblingElement();
    auto robot_location_Z = robot_location_Y->NextSiblingElement();

    auto robot_rotation_euler_X = robot_location_Z->NextSiblingElement();
    auto robot_rotation_euler_Y = robot_rotation_euler_X->NextSiblingElement();
    auto robot_rotation_euler_Z = robot_rotation_euler_Y->NextSiblingElement();

    auto GetDoubleArray = [](tinyxml2::XMLElement *element) {
        auto aryElem = element->FirstChildElement()->NextSiblingElement()->FirstChildElement();
        LOG_INFO(aryElem->FirstAttribute()->Value())
        const std::string str = aryElem->GetText();
        auto sVec = splitString(str, ' ');
        std::vector<double> dVec(sVec.size());
        for (int i = 0; i < sVec.size(); ++i) {
            dVec.at(i) = std::stod(sVec.at(i));
        }
        return dVec;
    };

    auto xVec = GetDoubleArray(robot_location_X);
    auto yVec = GetDoubleArray(robot_location_Y);
    auto zVec = GetDoubleArray(robot_location_Z);

    auto xEulerVec = GetDoubleArray(robot_rotation_euler_X);
    auto yEulerVec = GetDoubleArray(robot_rotation_euler_Y);
    auto zEulerVec = GetDoubleArray(robot_rotation_euler_Z);

    LOG_VAR(xVec.size(), yVec.size(), zVec.size())
    LOG_VAR(xEulerVec.size(), yEulerVec.size(), zEulerVec.size())

    std::vector<std::array<double, 6>> ary(xVec.size());

    for (int i = 0; i < xVec.size(); ++i) {
        auto &elem = ary.at(i);
        elem.at(0) = xVec.at(i);
        elem.at(1) = yVec.at(i);
        elem.at(2) = zVec.at(i);
        elem.at(3) = xEulerVec.at(i);
        elem.at(4) = yEulerVec.at(i);
        elem.at(5) = zEulerVec.at(i);
    }

    return ary;
}

tinyxml2::XMLNode *GetSimRobotScriptFromDEA(const std::string &filename, tinyxml2::XMLDocument &target) {
    const auto poseSeq = ReadPoseVecFromDEA(filename);

    tinyxml2::XMLDocument doc;
    static const double degToRad = M_PI / 180.0;

    auto scriptElem = doc.NewElement("script");

    auto loopElem = doc.NewElement("loop");
    loopElem->InsertEndChild(doc.NewText("1"));
    scriptElem->InsertEndChild(loopElem);

    auto autoStartElem = doc.NewElement("auto_start");
    autoStartElem->InsertEndChild(doc.NewText("1"));
    scriptElem->InsertEndChild(autoStartElem);

    auto trajectoryElem = doc.NewElement("trajectory");
    trajectoryElem->SetAttribute("id", 0);
    trajectoryElem->SetAttribute("type", "sim-traj");
    for (int i = 0; i < poseSeq.size(); ++i) {
        const auto &pose = poseSeq.at(i);
        const double timeStamp = i / 10.0;
        auto waypointElem = doc.NewElement("waypoint");

        auto timeElem = doc.NewElement("time");
        timeElem->InsertEndChild(doc.NewText(std::to_string(timeStamp).c_str()));
        waypointElem->InsertEndChild(timeElem);

        auto poseElem = doc.NewElement("pose");
        std::stringstream stream;
        stream << pose.at(0) << ' '
               << pose.at(1) << ' '
               << pose.at(2) << ' '
               << pose.at(3) * degToRad << ' '
               << pose.at(4) * degToRad << ' '
               << pose.at(5) * degToRad;
        poseElem->InsertEndChild(doc.NewText(stream.str().c_str()));
        waypointElem->InsertEndChild(poseElem);

        trajectoryElem->InsertEndChild(waypointElem);
    }
    scriptElem->InsertEndChild(trajectoryElem);

    return scriptElem->DeepClone(&target);
}

tinyxml2::XMLNode *GetSimRobotLinkFromSDF(const std::string &filename, tinyxml2::XMLDocument &target) {
    tinyxml2::XMLDocument robot;
    robot.LoadFile(filename.c_str());
    auto linkElem = robot.FirstChild()->FirstChild()->FirstChild();
    return linkElem->DeepClone(&target);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "generate_actor");
    LOG_PROCESS("start reading trajectory dea file")

    tinyxml2::XMLDocument doc;

    auto actorElem = doc.NewElement("actor");
    actorElem->SetAttribute("name", "sim_robot");

    auto linkElem = GetSimRobotLinkFromSDF(
            "/home/csl/ros_ws/sim-scene/src/scene/model/sdf/sim-robot.sdf", doc
    );
    actorElem->InsertEndChild(linkElem);

    auto scriptElem = GetSimRobotScriptFromDEA(
            "/home/csl/ros_ws/sim-scene/src/scene/model/trajectory/trajectory.dae", doc);
    actorElem->InsertEndChild(scriptElem);

    doc.InsertEndChild(actorElem);

    doc.SaveFile("/home/csl/ros_ws/sim-scene/src/scene/model/sdf/actor.sdf");

    LOG_PROCESS("save trajectory dea to file")
    ros::shutdown();
    return 0;
}
