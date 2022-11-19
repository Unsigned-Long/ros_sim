//
// Created by csl on 11/3/22.
//
#include "ros/ros.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "artwork/logger/logger.h"
#include "artwork/csv/csv.h"
#include <mav_msgs/conversions.h>
#include "sensor_msgs/Imu.h"

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

static const int64_t kNanoSecondsInSecond = 1000000000;

bool sim_running = false;

void callback(const sensor_msgs::ImuPtr &msg) {
    sim_running = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "firefly_pose_controller");
    ros::NodeHandle handle;

    LOG_PROCESS("loading trajectory...")
    auto waypoints = ReadPoseVecFromFile(
            "/home/csl/ros_ws/sim-scene/src/trajectory_controller/trajectory/trajectory.csv"
    );

    ros::Subscriber sub = handle.subscribe("/firefly/imu", 10, &callback);
    ROS_INFO("Wait for simulation to become ready...");

    while (!sim_running && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    LOG_PROCESS("start sim.")
    auto posePublisher = handle.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 10);

    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    msg->header.stamp = ros::Time::now();
    msg->points.resize(waypoints.size());
    msg->joint_names.push_back("base_link");
    int64_t time_from_start_ns = 0;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto &wp = waypoints[i];

        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W.x() = wp.x;
        trajectory_point.position_W.y() = wp.y;
        trajectory_point.position_W.z() = wp.z;

        Eigen::Quaterniond q(wp.qw, wp.qx, wp.qy, wp.qz);
        auto angles = q.toRotationMatrix().eulerAngles(0, 1, 1);
        trajectory_point.setFromYaw(angles(2));

        trajectory_point.time_from_start_ns = time_from_start_ns;

        LOG_VAR(time_from_start_ns, wp.x, wp.y, wp.z, angles(2))

        time_from_start_ns += static_cast<int64_t>(0);

        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
    }
    posePublisher.publish(msg);

    LOG_PROCESS("publish trajectory finished...")
    ros::spin();

    ros::shutdown();
    return 0;
}