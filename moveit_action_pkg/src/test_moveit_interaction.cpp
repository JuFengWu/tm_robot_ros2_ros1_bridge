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
  currentPistion[1]+=0.3;
  
  
  robotCmd->cartesian_move(currentPistion,false);
  std::cout<<"finish send cartesian command!"<<std::endl;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.01;
  primitive.dimensions[2] = 0.1;

  geometry_msgs::Pose boxPose;
  boxPose.orientation.w = 1.0;
  boxPose.position.x = 0.5;
  boxPose.position.y = -0.3;
  boxPose.position.z =  0.4;

  ros::WallDuration sleepTime(3);
  std::cout<<"add one box !"<<std::endl;
  robotCmd->add_solid_to_moveit(primitive,boxPose,"Box1");
  sleepTime.sleep();

  std::vector<double> planningPosition;
  planningPosition.resize(7);
  planningPosition[0] = 0.4975;
  planningPosition[1] = -0.4;
  for(int i=2;i<currentPistion.size();i++){
    planningPosition[i] = currentPistion[i];
  }

  robotCmd->cartesian_move(planningPosition,true);
  
  std::cout<<"robotCmd move commnad send!"<<std::endl;
  sleepTime.sleep();
  
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 0.1;
  primitive2.dimensions[1] = 0.01;
  primitive2.dimensions[2] = 0.1;

  geometry_msgs::Pose boxPose2;
  boxPose2.orientation.w = 1.0;
  boxPose2.position.x = -0.4;
  boxPose2.position.y = -0.3;
  boxPose2.position.z = 0.0;

  robotCmd->add_solid_to_moveit(primitive2,boxPose2,"Box2");

  std::cout<<"add all box !"<<std::endl;
  
  sleepTime.sleep();

  robotCmd->remove_soild_from_moveit("Box2");
  std::cout<<"remove  box2 !"<<std::endl;
  sleepTime.sleep();
  robotCmd->remove_soild_from_moveit("Box1");
  std::cout<<"remove  box1 !"<<std::endl;

  return 0;
}
