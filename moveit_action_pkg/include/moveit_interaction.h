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
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup;
      std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visualTools;
      const robot_state::JointModelGroup* jointModelGroup;
      const double jump_threshold =0.0;
      const double eef_step = 0.01;
      ros::Publisher planningSceneDiffPublisher;
      ros::NodeHandle nodeHandle;
      moveit_msgs::PlanningScene planningScene;

      void ros_initial();
  public:
    static double degree_to_rad(double degree);
    static std::vector<double> joint_degs_to_joint_rads(std::vector<double> jointDegrees);

    explicit RosMove(std::string robotName);

    bool joint_move(std::vector<double> jointTarget, bool isPlan);
    bool cartesian_move(std::vector<double> cartesianTarget, bool isPlan);
    bool cartesian_move(geometry_msgs::Pose cartesianTarget, bool isPlan);
    std::vector<double> get_current_end_effector_position();
    geometry_msgs::Pose current_end_effector_position();
    bool add_solid_to_moveit(shape_msgs::SolidPrimitive obj,geometry_msgs::Pose position,std::string objectName);
    bool remove_soild_from_moveit(std::string objectName);

  };

  class RobotJointStatePub{
  private:
    std::vector<double> currentJointPosition;
    std::vector<double> lastJointPosition;
    std::vector<std::string> jointNames;
    const int publishHz = 10;
    ros::Publisher chatterPub;
    bool isSettingJointPosition = false;
    void set_joint_name();
    bool isPublish = true;
  public:
    RobotJointStatePub(ros::NodeHandle nodeHandle,std::vector<double> initialJointPosition);
    void message_publish();
    bool set_new_joint_position(std::vector<double> newJointPosition);
    ~RobotJointStatePub(){
      isPublish = false;
    }
  };
  
}
