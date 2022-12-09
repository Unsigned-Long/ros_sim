#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <utility>
#include <tf/transform_broadcaster.h>
#include <ostream>
#include "gazebo_msgs/LinkStates.h"
#include "tf/transform_listener.h"
#include "eigen3/Eigen/Dense"
#include "artwork/logger/logger.h"
#include "sophus/se3.hpp"

struct Posed {
    ros::Time time;
    Sophus::SE3d pose;

    Posed() = default;

    Posed(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const ros::Time &time = ros::Time::now())
            : pose(q, t), time(time) {}

    explicit Posed(const Sophus::SE3d &pose, const ros::Time &time = ros::Time::now())
            : pose(pose), time(time) {}

    explicit Posed(const tf::StampedTransform &transform) {
        auto q = Eigen::Quaterniond(
                transform.getRotation().getW(),
                transform.getRotation().getX(),
                transform.getRotation().getY(),
                transform.getRotation().getZ()
        );
        auto t = Eigen::Vector3d(
                transform.getOrigin().getX(),
                transform.getOrigin().getY(),
                transform.getOrigin().getZ()
        );
        pose = Sophus::SE3d(q, t);
    }

    friend std::ostream &operator<<(std::ostream &os, const Posed &posed) {
        os << "time: " << posed.time.toSec()
           << " q: " << posed.pose.unit_quaternion().coeffs().transpose()
           << " t: " << posed.pose.translation().transpose();
        return os;
    }

    [[nodiscard]] nav_msgs::Odometry ToOdometryMsg(const std::string &frameId) const {
        const auto t = pose.translation();
        const auto q = pose.unit_quaternion();
        nav_msgs::Odometry odom;
        odom.pose.pose.position.x = t(0);
        odom.pose.pose.position.y = t(1);
        odom.pose.pose.position.z = t(2);

        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        odom.header.frame_id = "map";
        odom.header.stamp = time;
        odom.child_frame_id = frameId;
        return odom;
    }
};

void PublishGroundTruth(const gazebo_msgs::LinkStates::ConstPtr &msg,
                        const Posed &LtoBF, ros::Publisher &lidar_ground_truth_pub,
                        const Posed &ItoBF, ros::Publisher &imu_ground_truth_pub,
                        const Posed &CtoBF, ros::Publisher &camera_ground_truth_pub) {
    Posed BFtoM;
    ros::Time time = ros::Time::now();
    for (int i = 0; i < msg->name.size(); ++i) {
        if (msg->name.at(i) == "sim_robot::base_footprint") {
            const auto &BFPose = msg->pose.at(i);
            const auto &orientation = BFPose.orientation;
            const auto &position = BFPose.position;
            BFtoM = Posed(
                    Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z),
                    Eigen::Vector3d(position.x, position.y, position.z)
            );
            auto LtoM = Posed(BFtoM.pose * LtoBF.pose, time);
            auto ItoM = Posed(BFtoM.pose * ItoBF.pose, time);
            auto CtoM = Posed(BFtoM.pose * CtoBF.pose, time);
            lidar_ground_truth_pub.publish(LtoM.ToOdometryMsg("lidar"));
            imu_ground_truth_pub.publish(ItoM.ToOdometryMsg("imu"));
            camera_ground_truth_pub.publish(CtoM.ToOdometryMsg("camera"));
            break;
        }
    }
}

tf::StampedTransform GetTransform(const std::string &from, const std::string &to) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform(to, from, ros::Time(), ros::Duration(3.0));
    listener.lookupTransform(to, from, ros::Time(), transform);
    return transform;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_robot_state");
    ros::NodeHandle nh;

    Posed LtoBF(GetTransform("lidar", "base_footprint"));
    Posed ItoBF(GetTransform("imu", "base_footprint"));
    Posed CtoBF(GetTransform("camera", "base_footprint"));

    LOG_VAR(LtoBF)
    LOG_VAR(ItoBF)
    LOG_VAR(CtoBF)

    tf::TransformBroadcaster dynamicBroadcaster;
    auto base_footprint_to_map = nh.subscribe<nav_msgs::Odometry>(
            "/base_footprint_ground_truth", 10, [&dynamicBroadcaster](const nav_msgs::Odometry::ConstPtr &msg) {
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.frame_id = "map";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.header = msg->header;
                odom_trans.transform.translation.x = msg->pose.pose.position.x;
                odom_trans.transform.translation.y = msg->pose.pose.position.y;
                odom_trans.transform.translation.z = msg->pose.pose.position.z;

                odom_trans.transform.rotation = msg->pose.pose.orientation;

                dynamicBroadcaster.sendTransform(odom_trans);
            });

//    auto lidar_ground_truth_pub = nh.advertise<nav_msgs::Odometry>("/lidar_ground_truth", 100);
//    auto imu_ground_truth_pub = nh.advertise<nav_msgs::Odometry>("/imu_ground_truth", 100);
//    auto camera_ground_truth_pub = nh.advertise<nav_msgs::Odometry>("/camera_ground_truth", 100);

//    auto ground_truth_handler = nh.subscribe<gazebo_msgs::LinkStates>(
//            "/gazebo/link_states", 100,
//            [&LtoBF, &lidar_ground_truth_pub, &ItoBF, &imu_ground_truth_pub, &CtoBF, &camera_ground_truth_pub](
//                    auto &&PH1) {
//                return PublishGroundTruth(
//                        std::forward<decltype(PH1)>(PH1),
//                        LtoBF, lidar_ground_truth_pub,
//                        ItoBF, imu_ground_truth_pub,
//                        CtoBF, camera_ground_truth_pub
//                );
//            }
//    );

    ros::spin();
    ros::shutdown();

    return 0;
}