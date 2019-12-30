// Copyright 2019 Leo_Wu

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <iostream>
class JointTrajectoryAction{
 protected:
  ros::NodeHandle rosNode;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> actionServer;
  trajectory_msgs::JointTrajectory currentTrajectory;

  control_msgs::FollowJointTrajectoryResult result;
  ros::Publisher jointTrajectoryPub;
  const int tmRobotJointNumber = 6;

 public:
  explicit JointTrajectoryAction(std::string name) :
    actionServer(rosNode, name, boost::bind(&JointTrajectoryAction::execute_cb, this, _1), false) {
    ROS_INFO("initial JointTrajectoryAction");
    actionServer.start();
    jointTrajectoryPub = rosNode.advertise<trajectory_msgs::JointTrajectory>("tm_moveit_joint_trajectory", 1000);
  }

  ~JointTrajectoryAction(void) {
  }

  trajectory_msgs::JointTrajectory sort_trajecotry(trajectory_msgs::JointTrajectory moveitTrajectory) {
    trajectory_msgs::JointTrajectory tmRobotTrajecotry;
    tmRobotTrajecotry.points.resize(tmRobotJointNumber);
    for (int i= 0; i< moveitTrajectory.points.size(); i++) {
      for (int j= 0; j< moveitTrajectory.points[i].positions.size(); j++) {
        tmRobotTrajecotry.points[j].positions.push_back(moveitTrajectory.points[i].positions[j]);
      }
    }
    return tmRobotTrajecotry;
  }

  void publish_trajectory(trajectory_msgs::JointTrajectory trajectory) {
    auto tmRobotTrajecotry = sort_trajecotry(trajectory);
    jointTrajectoryPub.publish(tmRobotTrajecotry);
  }

  void print_trajectory(trajectory_msgs::JointTrajectory trajectory) {
    for (int i =0; i< trajectory.joint_names.size(); i++) {
      std::cout<< "the "<< i<< " name is "<< trajectory.joint_names[i]<< std::endl;
    }
    std::cout<< "the points number are "<< trajectory.points.size()<< std::endl;

    for (int i=0; i< trajectory.points.size(); i++) {
      std::cout<< trajectory.points[i].positions[0]<< ","<< trajectory.points[i].positions[1]<< ","<< trajectory.points[i].positions[2]<< ","<<
      trajectory.points[i].positions[3]<< ","<< trajectory.points[i].positions[4]<< ","<< trajectory.points[i].positions[5]<< std::endl;
    }
  }

  void execute_cb(const control_msgs::FollowJointTrajectoryGoalConstPtr msg){
    if ( !(*msg).trajectory.points.empty() ) {
      currentTrajectory = (*msg).trajectory;
      print_trajectory(currentTrajectory);
      publish_trajectory(currentTrajectory);
      ros::spinOnce();
    } else {
      ROS_ERROR("joint trajectory from moveit is null!");
    }
    if (actionServer.isActive()){
      actionServer.setSucceeded();
    }
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "tm_joint_trajectory_action");

  ROS_INFO("in JointTrajectoryAction main!!!!!!!");

  JointTrajectoryAction jointTrajectoryAction("tm_arm_controller/follow_joint_trajectory");
  ros::spin();

  return 0;
}

