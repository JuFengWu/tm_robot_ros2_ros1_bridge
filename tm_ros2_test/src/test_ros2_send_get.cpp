#include "../include/moveit_bridge_ros1.h"
#include<iostream>

void Ros1MoveitBridge::joint_trajectory_sub_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
  ROS_INFO("Hear something");
  this->trajectory.points = msg->points;
  this->isGetResult = true;
}

void Ros1MoveitBridge::current_position_sub_callback(const geometry_msgs::Pose::ConstPtr& msg){
  ROS_INFO("Hear something");
  this->current_position = *msg;
}

Ros1MoveitBridge::Ros1MoveitBridge(ros::NodeHandle nodeHandle){
  isGetResult = false;
  jointStatesChatter = nodeHandle.advertise<sensor_msgs::JointState>("tm_moveit_joint_states", 1000);
  jointTargetChatter = nodeHandle.advertise<sensor_msgs::JointState>("tm_moveit_joint_target", 1000);
  cartesianChatter = nodeHandle.advertise<geometry_msgs::Pose>("tm_moveit_cartiesan_target", 1000);
  ros::Subscriber jointTrajectorySub = nodeHandle.subscribe("tm_moveit_joint_trajectory", 1000, &Ros1MoveitBridge::joint_trajectory_sub_callback, this);
  ros::Subscriber currentPositionSub = nodeHandle.subscribe("tm_moveit_current_position",1000, &Ros1MoveitBridge::current_position_sub_callback, this);
}
void Ros1MoveitBridge::set_current_joint(std::vector<double> jointPosition){
  sensor_msgs::JointState msg;
  msg.header.stamp =  ros::Time::now();
  //msg.name = jointNames; 
  msg.position = jointPosition;
  jointStatesChatter.publish(msg);
  ros::spinOnce();
}

trajectory_msgs::JointTrajectory Ros1MoveitBridge::get_trajectories(std::vector<double> jointTarget){
  sensor_msgs::JointState msg;
  msg.header.stamp =  ros::Time::now();
  //msg.name = jointNames; 
  msg.position = jointTarget;
  jointTargetChatter.publish(msg);
  ros::spinOnce();
  ros::Rate loop_rate(500);// 2ms
  while(isGetResult){
    loop_rate.sleep();
  }
  isGetResult = false;
  return trajectory;
  
}
trajectory_msgs::JointTrajectory Ros1MoveitBridge::get_trajectories(geometry_msgs::Pose eeTarget){
  geometry_msgs::Pose msg;
  msg = eeTarget;
  cartesianChatter.publish(msg);
  ros::spinOnce();
  ros::Rate loop_rate(500);// 2ms
   while(isGetResult){
    loop_rate.sleep();
  }
  isGetResult = false;
  return trajectory;
}

geometry_msgs::Pose Ros1MoveitBridge::get_current_position(){
  return current_position;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tm_move_api");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nodeHandle;

  std::unique_ptr<MoveitBridgeInterface> moveitBrdige = std::make_unique<Ros1MoveitBridge>(nodeHandle);

  std::vector<double> initialJointPosition{0.0,0.0,90.0,0.0,90.0,0.0};
  moveitBrdige->set_current_joint(initialJointPosition);
  
  std::vector<double> targetJointPosition1{21.0,32.0,90.0,43.0,90.0,8.0};
  auto trajectories = moveitBrdige->get_trajectories(targetJointPosition1);

  std::cout<<"trajectories are :"<<std::endl;
  for(int i=0; i<trajectories.points[0].positions.size();i++){
    for(int j=0;i<trajectories.points.size();j++){
        std::cout<<"J"<<j<<":"<<trajectories.points[j].positions[i]<<",";
    }
    std::cout<<std::endl;
  }

  auto currentPosition = moveitBrdige->get_current_position();
  
  geometry_msgs::Pose cmdPosition = currentPosition;
  cmdPosition.position.x +=0.5;
  cmdPosition.position.y +=0.5;
  cmdPosition.position.z +=0.5;

  trajectories = moveitBrdige->get_trajectories(cmdPosition);
  std::cout<<"trajectories are :"<<std::endl;
  for(int i=0; i<trajectories.points[0].positions.size();i++){
    for(int j=0;i<trajectories.points.size();j++){
        std::cout<<"J"<<j<<":"<<trajectories.points[j].positions[i]<<",";
    }
    std::cout<<std::endl;
  }

}