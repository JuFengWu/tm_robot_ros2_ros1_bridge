#pragma once
#include "geometry_msgs/Pose.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include <memory>

class MoveitBridgeInterface
{
public:
  virtual void set_current_joint(std::vector<double> jointPosition)=0;
  virtual trajectory_msgs::JointTrajectory get_trajectories(std::vector<double> jointTarget)=0;
  virtual trajectory_msgs::JointTrajectory get_trajectories(geometry_msgs::Pose jointTarget)=0;
  virtual geometry_msgs::Pose get_current_position()=0;
};

