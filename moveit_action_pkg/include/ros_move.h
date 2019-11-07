#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <thread>
namespace robot_move_api{
  class RosMove
  {
  private:
      std::string PLANNING_GROUP ;
      void ros_initial();
      moveit::planning_interface::MoveGroupInterface *_move_group;
      moveit_visual_tools::MoveItVisualTools *_visual_tools;
      const robot_state::JointModelGroup* _joint_model_group;
      const double jump_threshold =0.0;
      const double eef_step = 0.01;
  public:
    RosMove(std::string robotName){
      ros_initial();
    };
    bool joint_move(std::vector<double> jointTarget, bool isPlan);
    bool cartesian_move(std::vector<double> cartesianTarget, bool isPlan);
    int get_joint_number();
    std::vector<double> get_current_end_effector_position();
    std::vector<double> get_current_joint_position();
    static double degree_to_rad(double degree);
    static std::vector<double> joint_degs_to_joint_rads(std::vector<double> jointDegrees);
  };

  class RobotJointStatePub{
  private:
    std::vector<double> currentJointPosition;
    std::vector<double> lastJointPosition;
    std::vector<std::string> jointNames;
    const int publishHz = 10;
    ros::Publisher chatter_pub;
    bool isSettingJointPosition = false;
    void set_joint_name();
    bool isPublish = true;
  public:
    RobotJointStatePub(ros::NodeHandle node_handle,std::vector<double> initialJointPosition);
    void message_publish();
    bool set_new_joint_position(std::vector<double> newJointPosition);
    ~RobotJointStatePub(){
      isPublish = false;
    }
  };
  
}