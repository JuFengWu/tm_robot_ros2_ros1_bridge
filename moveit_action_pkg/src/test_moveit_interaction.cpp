#include "../include/moveit_interaction.h"
#include<iostream>
#include <memory>
#include<string>


int main(int argc, char** argv){
  ros::init(argc, argv, "tm_move_api");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle NodeHandle;

  std::vector<double> initialJointPosition{0.0,0.0,90.0,0.0,90.0,0.0};

  auto initialJointPositionRad = robot_move_api::RosMove::joint_degs_to_joint_rads(initialJointPosition);
  std::unique_ptr<robot_move_api::RobotJointStatePub> robotJointStatePub = std::make_unique<robot_move_api::RobotJointStatePub>(NodeHandle,initialJointPositionRad);

  std::unique_ptr<robot_move_api::RosMove> robotCmd = std::make_unique<robot_move_api::RosMove>("tm_robot");

  std::vector<double> jointTarget{21.0,32.0,90.0,43.0,90.0,8.0};
  
  auto jointTargetRad = robot_move_api::RosMove::joint_degs_to_joint_rads(jointTarget);
  
  robotCmd->joint_move(jointTargetRad,false);

  std::cout<<"finish send joint command!"<<std::endl;

  auto currentPistion = robotCmd->get_current_end_effector_position();
  
  std::cout<<"current position is:";
  for(int i=0;i<currentPistion.size();i++){
    std::cout<<currentPistion[i]<<",";
  }
  currentPistion[1]+=0.1;
  currentPistion[2]-=0.1;
  
  robotCmd->cartesian_move(currentPistion,false);
  std::cout<<"finish send cartesian command!"<<std::endl;
  return 0;
}