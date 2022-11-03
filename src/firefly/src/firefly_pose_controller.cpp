//
// Created by csl on 11/3/22.
//
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "artwork/logger/logger.h"
#include "artwork/csv/csv.h"

struct Pose {
    double x, y, z;
    double qx, qy, qz, qw;

    friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
        os << "x: " << pose.x << " y: " << pose.y << " z: " << pose.z << " qx: " << pose.qx << " qy: " << pose.qy
           << " qz: " << pose.qz << " qw: " << pose.qw;
        return os;
    }

    [[nodiscard]] std::vector<double> pos() const {
        return {x, y, z};
    }
};

std::vector<Pose> ReadPoseVecFromFile(const std::string &str) {
    LOG_PROCESS("read pose sequence from file: ", str)
    auto poseSeq = ns_csv::CSVReader::read<CSV_STRUCT(Pose, x, y, z, qx, qy, qz, qw) >(str, ',');
    LOG_INFO("pose sequence size: ", poseSeq.size())
    return poseSeq;
}

void IsFirFlyArrived(const geometry_msgs::Pose::ConstPtr &msg) {
    auto pos = std::vector<double>(3);
    ros::param::get("cur_tar_pos", pos);
    double dx = msg->position.x - pos[0];
    double dy = msg->position.y - pos[1];
    double dz = msg->position.z - pos[2];
    double dis = std::sqrt(dx * dx + dy * dy + dz * dz);
    LOG_PLAINTEXT("'cur pos': ", msg->position.x, ", ", msg->position.y, ", ", msg->position.z,
                  ", 'tar pos': ", pos[0], ", ", pos[1], ", ", pos[2], ", 'dist': ", dis)
    if (dis < 0.5) {
        ros::param::set("is_arrived", true);
        LOG_INFO("firefly has arrived the target position, starting to move to the next position.")
    } else {
        ros::param::set("is_arrived", false);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "firefly_pose_controller");
    ros::NodeHandle handle;

    LOG_PROCESS("loading trajectory...")
    auto poseSeq = ReadPoseVecFromFile(
            "/home/csl/ros_ws/sim-scene/src/trajectory_controller/trajectory/trajectory.csv"
    );

    LOG_PROCESS("start sim.")
    int poseSeqIdx = 0;
    ros::param::set("is_arrived", false);
    ros::param::set("cur_tar_pos", poseSeq.front().pos());

    auto posePublisher = handle.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 10);
    auto mposeMonitor = handle.subscribe<geometry_msgs::Pose>("/firefly/ground_truth/pose", 10, IsFirFlyArrived);

    ros::Rate r(1);

    while (ros::ok()) {
        bool isArrived;
        ros::param::get("is_arrived", isArrived);

        if (isArrived) {
            isArrived = false;
            poseSeqIdx += 10;
            if (poseSeqIdx >= poseSeq.size()) {
                break;
            } else {
                ros::param::set("cur_tar_pos", poseSeq.at(poseSeqIdx).pos());
            }
        }

        const auto &curPose = poseSeq.at(poseSeqIdx);

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = curPose.x;
        poseStamped.pose.position.y = curPose.y;
        poseStamped.pose.position.z = curPose.z;
        poseStamped.pose.orientation.x = curPose.qx;
        poseStamped.pose.orientation.y = curPose.qy;
        poseStamped.pose.orientation.z = curPose.qz;
        poseStamped.pose.orientation.w = curPose.qw;
        posePublisher.publish(poseStamped);

        r.sleep();
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}