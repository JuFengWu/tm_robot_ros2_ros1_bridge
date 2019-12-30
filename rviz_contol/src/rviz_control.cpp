// Copyright 2019 Leo_Wu




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>

#include <thread>

#include "boost/date_time/posix_time/posix_time.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "../include/modle_interface.hpp"



class RvizControl{
 private:
  ros::NodeHandle nodeHandle;
  ros::Subscriber interactiveReactionSub;

  ros::Publisher eePositionTalker;
  ros::Publisher jointTargetTalker;

  const robot_state::JointModelGroup* jointModelGroup;
  std::unique_ptr <moveit::planning_interface::MoveGroupInterface> moveGroup;
  std::string planningGroup;

  void interactive_reaction(const geometry_msgs::Pose & markerPose);

  bool robot_inverse_kinematics
    (geometry_msgs::Pose markerPose, std::vector< double > &jointValues);
  geometry_msgs::Pose get_ee_postion();

  void send_ee_postion_to_marker(geometry_msgs::Pose eePostion);
  void send_joint_position_to_robot(std::vector<double> jointValues);
 public:
  void pass_robot_position_to_marker();
  RvizControl(std::string planningGroup);
  static void wait_connect_success(ros::Publisher publisher);
};

RvizControl::RvizControl(std::string planningGroup):
 planningGroup(planningGroup)
 ,moveGroup(std::make_unique<moveit::planning_interface::MoveGroupInterface>(planningGroup)) {

  jointModelGroup = moveGroup->getCurrentState()->getJointModelGroup(planningGroup);
  interactiveReactionSub = nodeHandle.subscribe("tm_interactive_reaction", 1000, &RvizControl::interactive_reaction, this);

  std::thread(&RvizControl::pass_robot_position_to_marker, this).detach();

  eePositionTalker = nodeHandle.advertise<geometry_msgs::Pose>("tm_current_ee_position", 1000);
  jointTargetTalker = nodeHandle.advertise<sensor_msgs::JointState>("tm_joint_target", 1000);
}

void RvizControl::interactive_reaction(const geometry_msgs::Pose & markerPose) {
  std::vector<double> jointValues;
  if (robot_inverse_kinematics(markerPose, jointValues)) {
    send_joint_position_to_robot(jointValues);
  } else {
    std::cout<< "can not cal IK"<< std::endl;
  }
}

void RvizControl::pass_robot_position_to_marker() {
  ros::Rate loopRate(5);
  while (ros::ok()) {
    auto eePostion = get_ee_postion();
    send_ee_postion_to_marker(eePostion);
    ros::spinOnce();
    loopRate.sleep();
  }
}

bool RvizControl::robot_inverse_kinematics(geometry_msgs::Pose markerPose, std::vector< double > &jointValues) {
  robot_state::RobotState robotState(*moveGroup->getCurrentState());
  jointModelGroup = moveGroup->getCurrentState()->getJointModelGroup(planningGroup);
  bool isIkSuccess = robotState.setFromIK(jointModelGroup, markerPose);
  if (isIkSuccess) {
    robotState.copyJointGroupPositions(jointModelGroup, jointValues);
    return true;
  }
  return false;
}

geometry_msgs::Pose RvizControl::get_ee_postion() {
  robot_state::RobotState robotState(*moveGroup->getCurrentState());
  geometry_msgs::Pose endEffectorStatePose;

  const Eigen::Affine3d &endEffectorState = robotState.getGlobalLinkTransform("tip_link");
  tf::poseEigenToMsg(endEffectorState, endEffectorStatePose);

  return endEffectorStatePose;
}

void RvizControl::wait_connect_success(ros::Publisher publisher) {
  ros::Rate loopRate(10);  // 100ms
  while (publisher.getNumSubscribers()== 0) {
    std::cout<< "wait connect success!"<< std::endl;
    loopRate.sleep();
  }
}

void RvizControl::send_ee_postion_to_marker(geometry_msgs::Pose eePostion) {
  RvizControl::wait_connect_success(eePositionTalker);
  eePositionTalker.publish(eePostion);
}
void RvizControl::send_joint_position_to_robot(std::vector<double> jointValues) {
  sensor_msgs::JointState msg;

  for (auto& n : jointValues) {
    msg.position.push_back(n);
  }
  RvizControl::wait_connect_success(jointTargetTalker);

  jointTargetTalker.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tm_rviz_control");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  std::string planningGroup = "tm_arm";
  std::unique_ptr<RvizControl> rvizControl = std::make_unique<RvizControl>(planningGroup);

  std::cout<< "rviz_control started!"<< std::endl;

  ros::waitForShutdown();

  return 0;
}
