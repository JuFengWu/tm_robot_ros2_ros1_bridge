#include "../include/ros_move.h"
#include<iostream>
#include <memory>
#include<string>
namespace robot_move_api{
    void RosMove::ros_initial(){
        
        
        
        PLANNING_GROUP = "tm_arm";
        _move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        _joint_model_group =_move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        _visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");      
        
    }
    bool RosMove::joint_move(std::vector<double> jointTarget, bool isPlan){
        _move_group->setJointValueTarget(jointTarget);
        bool success;
        if(isPlan){
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          success = (_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);            
          ROS_INFO("B");
        }
        else{        
          _move_group->move();
          success = true;
        }
        return success;
    }

    bool RosMove::cartesian_move(std::vector<double> cartesianTarget, bool isPlan){
       
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.w = cartesianTarget[3];
        target_pose1.orientation.x = cartesianTarget[4];
        target_pose1.orientation.y = cartesianTarget[5];
        target_pose1.orientation.z = cartesianTarget[6];
        target_pose1.position.x = cartesianTarget[0];
        target_pose1.position.y = cartesianTarget[1];
        target_pose1.position.z = cartesianTarget[2];
        std::vector<geometry_msgs::Pose> waypoint;
        waypoint.push_back(target_pose1);
        moveit_msgs::RobotTrajectory trajectory;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        //bool success = (_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //double fraction = _move_group->computeCartesianPath(waypoint, eef_step, jump_threshold, trajectory);
        _move_group->computeCartesianPath(waypoint, eef_step, jump_threshold, trajectory);

        if(!isPlan){
            my_plan.trajectory_ = trajectory;
            _move_group->execute(my_plan);
        }
        return true;
    }
    int RosMove::get_joint_number(){
        std::vector<double> joint_group_positions;
        moveit::core::RobotStatePtr current_state = _move_group->getCurrentState();
        current_state->copyJointGroupPositions(_joint_model_group, joint_group_positions);
        return joint_group_positions.size();
    }
    std::vector<double> RosMove::get_current_end_effector_position(){
        geometry_msgs::Pose pose = _move_group->getCurrentPose().pose;
        std::vector<double> endEffectorPosition;
        endEffectorPosition.push_back(pose.position.x);
        endEffectorPosition.push_back(pose.position.y);
        endEffectorPosition.push_back(pose.position.z);
        endEffectorPosition.push_back(pose.orientation.w);
        endEffectorPosition.push_back(pose.orientation.x);
        endEffectorPosition.push_back(pose.orientation.y);
        endEffectorPosition.push_back(pose.orientation.z);
        return endEffectorPosition;
    }
    std::vector<double> RosMove::get_current_joint_position(){
        std::vector<double> joint_group_positions;
        moveit::core::RobotStatePtr current_state = _move_group->getCurrentState();
        current_state->copyJointGroupPositions(_joint_model_group, joint_group_positions);
        return joint_group_positions;
    }
    double RosMove::degree_to_rad(double degree){
        return degree/180.0*M_PI;
    }
    std::vector<double> RosMove::joint_degs_to_joint_rads(std::vector<double> jointDegrees){
      std::vector<double> jointRad;
      for (double degree : jointDegrees){
        jointRad.push_back(RosMove::degree_to_rad(degree));
      }
      return jointRad;
    }

    RobotJointStatePub::RobotJointStatePub(ros::NodeHandle node_handle,std::vector<double> initialJointPosition){
      chatter_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1000);

      set_joint_name();
      set_new_joint_position(initialJointPosition);
      set_new_joint_position(initialJointPosition);// make sure lastJointPosition have value

      std::cout<<"finish initial robot state"<<std::endl;

      std::thread(&RobotJointStatePub::message_publish,this).detach();
    }
    void RobotJointStatePub::message_publish(){
      ros::Rate loop_rate(publishHz);  
      while (ros::ok() && isPublish){
        sensor_msgs::JointState msg;
        msg.header.stamp =  ros::Time::now();
        msg.name = jointNames;
        if(isSettingJointPosition){
            msg.position = lastJointPosition;
        }
        msg.position = currentJointPosition;
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
      }
      std::cout<<"end publish"<<std::endl;
    }
    void RobotJointStatePub::set_joint_name(){
      jointNames.push_back("shoulder_1_joint");
      jointNames.push_back("shoulder_2_joint");
      jointNames.push_back("elbow_1_joint");
      jointNames.push_back("wrist_1_joint");
      jointNames.push_back("wrist_2_joint");
      jointNames.push_back("wrist_3_joint");
      jointNames.push_back("finger_joint");
      jointNames.push_back("left_inner_knuckle_joint");
      jointNames.push_back("left_inner_finger_joint");
      jointNames.push_back("right_outer_knuckle_joint");
      jointNames.push_back("right_inner_knuckle_joint");
      jointNames.push_back("right_inner_finger_joint");
    }
    bool RobotJointStatePub::set_new_joint_position(std::vector<double> newJointPosition){
      if(newJointPosition.size()!=6){
        std::cout<<"input error~"<<std::endl;
        return false;
      }
      isSettingJointPosition = true;
      lastJointPosition.clear();
      lastJointPosition.assign(currentJointPosition.begin(), currentJointPosition.end());
      currentJointPosition.clear();
      currentJointPosition.assign(newJointPosition.begin(),newJointPosition.end());
      currentJointPosition.push_back(0.0);
      currentJointPosition.push_back(0.0);
      currentJointPosition.push_back(0.0);
      currentJointPosition.push_back(0.0);
      currentJointPosition.push_back(0.0);
      currentJointPosition.push_back(0.0);
      isSettingJointPosition = false;
      return true;
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tm_move_api");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node_handle;

  std::vector<double> initialJointPosition{0.0,0.0,90.0,0.0,90.0,0.0};

  auto initialJointPositionRad = robot_move_api::RosMove::joint_degs_to_joint_rads(initialJointPosition);
  std::unique_ptr<robot_move_api::RobotJointStatePub> robotJointStatePub = std::make_unique<robot_move_api::RobotJointStatePub>(node_handle,initialJointPositionRad);

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