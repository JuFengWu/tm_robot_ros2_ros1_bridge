#include "geometry_msgs/Pose.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include <memory>
#include <iostream>


void current_position_sub_callback(const geometry_msgs::Pose::ConstPtr& msg){

  std::cout<<"get cmdPosition which is";
  std::cout<<msg->position.x <<","<< msg->position.y<<","<<msg->position.z<<std::endl;
}

void joint_trajectory_sub_callback(const sensor_msgs::JointState::ConstPtr& msg){
  std::cout<<"get joint target which is";
  for(int i=0;i<6;i++){
        std::cout<<msg->position[i]<<",";
    }
  std::cout<<std::endl;
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "test_ros2_send_get");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  ros::NodeHandle nodeHandle;

  auto poseChatter = nodeHandle.advertise<geometry_msgs::Pose>("bridge_pose_to_ros2", 1000);
  auto trajectoryChatter = nodeHandle.advertise<trajectory_msgs::JointTrajectory>("bridge_trajectory_to_ros2", 1000);
  auto jointTrajectorySub = nodeHandle.subscribe("bridge_jointState_to_ros1", 1000,joint_trajectory_sub_callback);
  auto currentPositionSub = nodeHandle.subscribe("bridge_pose_to_ros1",1000, current_position_sub_callback);

  ros::Rate loop_rate(1);


  int counter=0;
  trajectory_msgs::JointTrajectory JointTrajectoryMsg;
  while (ros::ok())
  {
    JointTrajectoryMsg.points.resize(2);
    
    for(unsigned int i=0; i<JointTrajectoryMsg.points.size();i++){
      JointTrajectoryMsg.points[i].positions.resize(2);
      for(unsigned int j=0;j<JointTrajectoryMsg.points[i].positions.size();j++){
        JointTrajectoryMsg.points[i].positions[j]=counter;
      }
    }
    trajectoryChatter.publish(JointTrajectoryMsg);

    geometry_msgs::Pose cmdPosition;
    cmdPosition.position.x =counter;
    cmdPosition.position.y =counter;
    cmdPosition.position.z =counter;
    poseChatter.publish(cmdPosition);

    counter++;
    if(counter==360){
        counter =0;
    }
    ROS_INFO("Pub something");
    
    loop_rate.sleep();
  }


  return 0;
}