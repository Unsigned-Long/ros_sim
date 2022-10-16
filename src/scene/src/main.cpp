//
// Created by csl on 10/16/22.
//

#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/TFMessage.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "scene_node");
  ros::NodeHandle handler;
  auto statePublisher = handler.advertise<tf2_msgs::TFMessage>("/tf", 10);
  ros::Rate r(5);

  double yPos = 0.0;
  while (ros::ok()) {
    tf2_msgs::TFMessage msg;
    msg.transforms.resize(1);
    auto &transform = msg.transforms.front();
    transform.header.stamp = ros::Time::now();
    transform.header.seq = 100;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    tf2::Quaternion qtn;
    transform.transform.rotation.x = qtn.getX();
    transform.transform.rotation.y = qtn.getY();
    transform.transform.rotation.z = qtn.getZ();
    transform.transform.rotation.w = qtn.getW();
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = yPos;
    transform.transform.translation.z = 0.0;

    yPos += 0.1;
    statePublisher.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
