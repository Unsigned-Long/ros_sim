#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_robot_state");
    ros::NodeHandle nh;
    tf::TransformBroadcaster broadcaster;

    auto base_footprint_to_map = nh.subscribe<nav_msgs::Odometry>(
            "/base_footprint_ground_truth", 10, [&broadcaster](const nav_msgs::Odometry::ConstPtr &msg) {
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.frame_id = "map";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.header = msg->header;
                odom_trans.transform.translation.x = msg->pose.pose.position.x;
                odom_trans.transform.translation.y = msg->pose.pose.position.y;
                odom_trans.transform.translation.z = msg->pose.pose.position.z;

                odom_trans.transform.rotation = msg->pose.pose.orientation;

                broadcaster.sendTransform(odom_trans);
            });

    ros::spin();
    ros::shutdown();

    return 0;
}