#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim_robot_state");
  ros::NodeHandle n;

  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);

  const double degree = M_PI / 180;

  // robot state
  double angle = 0;

  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "base_footprint";

  while (ros::ok()) {
    // update transform
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = cos(angle) * 2;
    odom_trans.transform.translation.y = sin(angle) * 2;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle + M_PI / 2);

    // send the joint state and transform
    broadcaster.sendTransform(odom_trans);

    // Create new robot state
    angle += degree / 4;

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }
  ros::shutdown();

  return 0;
}