/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ostream>

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
};

std::vector<Pose> ReadPoseVecFromFile(const std::string &str) {
    LOG_PROCESS("read pose sequence from file: ", str)
    auto poseSeq = ns_csv::CSVReader::read<CSV_STRUCT(Pose, x, y, z, qx, qy, qz, qw) >(str, ',');
    LOG_INFO("pose sequence size: ", poseSeq.size())
    return poseSeq;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveGroupInterface");
    ros::NodeHandle node_handle;

    auto poseSeq = ReadPoseVecFromFile(
            "/home/csl/ros_ws/sim-scene/src/trajectory_controller/trajectory/trajectory.csv"
    );

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to startState an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // -----
    // Setup
    // -----

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangeably.
    static const std::string PLANNING_GROUP = "arm";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface moveGroupInterface(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup *jointModelGroup =
            moveGroupInterface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // -------------
    // Visualization
    // -------------

    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools("base");
    visualTools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visualTools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d textPose = Eigen::Isometry3d::Identity();
    textPose.translation().z() = 1.0;
    visualTools.publishText(textPose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visualTools.trigger();

    // -------------------------
    // Getting Basic Information
    // -------------------------

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", moveGroupInterface.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", moveGroupInterface.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(moveGroupInterface.getJointModelGroupNames().begin(),
              moveGroupInterface.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // --------------
    // Start the demo
    // --------------
    visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to startState the trajectory");

    // ---------------------
    // Moving to a pose goal
    // ---------------------
    //
    // If you do not want to inspect the planned trajectory,
    // the following is a more robust combination of the two-step plan+execute pattern shown above
    // and should be preferred. Note that the pose goal we had set earlier is still active,
    // so the robot will try to move to that goal.

    moveGroupInterface.setMaxVelocityScalingFactor(0.1);
    moveGroupInterface.setMaxAccelerationScalingFactor(0.1);

    for (int i = 0; i < poseSeq.size(); i += 5) {
        const auto &p = poseSeq.at(i);

        geometry_msgs::Pose targetPose;
        targetPose.orientation.x = p.qx;
        targetPose.orientation.y = p.qy;
        targetPose.orientation.z = p.qz;
        targetPose.orientation.w = p.qw;
        targetPose.position.x = p.x;
        targetPose.position.y = p.y;
        targetPose.position.z = p.z;

        moveGroupInterface.setPoseTarget(targetPose);

        LOG_PROCESS("moving to pose: ", p)

        moveGroupInterface.move();

        if(i==0){
          visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the trajectory");
        }
    }


    ros::shutdown();
    return 0;
}
