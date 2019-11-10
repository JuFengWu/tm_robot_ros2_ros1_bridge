#include "../include/moveit_bridge_ros1.h"
#include<iostream>

void Ros1MoveitBridge::joint_trajectory_sub_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
  ROS_INFO("Hear trajectory");
  this->trajectory.points = msg->points;
  this->isGetResult = true;
}

void Ros1MoveitBridge::current_position_sub_callback(const geometry_msgs::Pose::ConstPtr& msg){
  this->current_position = *msg;
}

Ros1MoveitBridge::Ros1MoveitBridge(){
  isGetResult = false;
  jointStatesChatter = nodeHandle.advertise<sensor_msgs::JointState>("tm_moveit_joint_states", 1000);
  jointTargetChatter = nodeHandle.advertise<sensor_msgs::JointState>("tm_moveit_joint_target", 1000);
  cartesianChatter = nodeHandle.advertise<geometry_msgs::Pose>("tm_moveit_cartiesan_target", 1000);
  jointTrajectorySub = nodeHandle.subscribe("tm_moveit_joint_trajectory", 1000, &Ros1MoveitBridge::joint_trajectory_sub_callback, this);
  currentPositionSub = nodeHandle.subscribe("tm_moveit_current_position",1000, &Ros1MoveitBridge::current_position_sub_callback, this);
}
void Ros1MoveitBridge::set_current_joint(std::vector<double> jointPosition){
  sensor_msgs::JointState msg;
  msg.header.stamp =  ros::Time::now();
  //msg.name = jointNames; 
  msg.position = jointPosition;
  Ros1MoveitBridge::wait_connect_success(jointStatesChatter);
  jointStatesChatter.publish(msg);
  ros::spinOnce();
}

void Ros1MoveitBridge::wait_connect_success(ros::Publisher publisher){
  ros::Rate loop_rate(10);//100ms
  while(publisher.getNumSubscribers()== 0){
    std::cout<<"wait connect success!"<<std::endl;
    loop_rate.sleep();
  }
}

trajectory_msgs::JointTrajectory Ros1MoveitBridge::get_trajectories(std::vector<double> jointTarget){
  sensor_msgs::JointState msg;
 
  msg.header.stamp =  ros::Time::now();
  
  //msg.name = jointNames; 
  msg.position = jointTarget;

  Ros1MoveitBridge::wait_connect_success(jointTargetChatter);
  
  jointTargetChatter.publish(msg);
  
  ros::spinOnce();
  
  ros::Rate loop_rate(5);//200ms
  
  while(!isGetResult){
    loop_rate.sleep();
    std::cout<<"wait result"<<std::endl;
  }
  isGetResult = false;
  return trajectory;
  
}
trajectory_msgs::JointTrajectory Ros1MoveitBridge::get_trajectories(geometry_msgs::Pose eeTarget){
  geometry_msgs::Pose msg;
  msg = eeTarget;
  Ros1MoveitBridge::wait_connect_success(cartesianChatter);
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
  ros::init(argc, argv, "test_ros2_send_get");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  
  std::unique_ptr<MoveitBridgeInterface> moveitBrdige = std::make_unique<Ros1MoveitBridge>();
  
  std::vector<double> initialJointPosition{0.0,0.0,90.0,0.0,90.0,0.0};

  moveitBrdige->set_current_joint(initialJointPosition);
  
  std::vector<double> targetJointPosition1{21.0,32.0,90.0,43.0,90.0,8.0};
  auto trajectories = moveitBrdige->get_trajectories(targetJointPosition1);

  std::cout<<"trajectories are :"<<std::endl;
  for(int i=0; i<trajectories.points.size();i++){
    std::cout<<"joint "<<i<<" trajectory points are:";
    for(int j=0;j<trajectories.points[i].positions.size();j++){

        std::cout<<trajectories.points[i].positions[j]<<",";
    }
    std::cout<<std::endl;
  }

  auto currentPosition = moveitBrdige->get_current_position();
  
  geometry_msgs::Pose cmdPosition = currentPosition;
  cmdPosition.position.x +=0.5;
  cmdPosition.position.y +=0.5;
  cmdPosition.position.z +=0.5;

  trajectories = moveitBrdige->get_trajectories(cmdPosition);
  std::cout<<"trajectories_2 are :"<<std::endl;
  for(int i=0; i<trajectories.points.size();i++){
    std::cout<<"joint "<<i<<" trajectory points are:";
    
    for(int j=0;j<trajectories.points[i].positions.size();j++){

        std::cout<<trajectories.points[i].positions[j]<<",";
    }
    std::cout<<std::endl;
  }

}