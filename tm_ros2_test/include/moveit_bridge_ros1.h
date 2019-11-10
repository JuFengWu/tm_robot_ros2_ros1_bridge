
#include "moveit_bridge_interface.h"


class Ros1MoveitBridge : public MoveitBridgeInterface{
private:
  ros::Publisher jointStatesChatter;
  ros::Publisher jointTargetChatter;
  ros::Publisher cartesianChatter;

  ros::Subscriber jointTrajectorySub;
  ros::Subscriber currentPositionSub;

  trajectory_msgs::JointTrajectory trajectory;
  geometry_msgs::Pose current_position;
  bool isGetResult;
  ros::NodeHandle nodeHandle;

  void joint_trajectory_sub_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void current_position_sub_callback(const geometry_msgs::Pose::ConstPtr& msg);
public:
  Ros1MoveitBridge();
  void set_current_joint(std::vector<double> jointPosition) override;
  trajectory_msgs::JointTrajectory get_trajectories(std::vector<double> jointTarget) override;
  trajectory_msgs::JointTrajectory get_trajectories(geometry_msgs::Pose jointTarget) override;
  geometry_msgs::Pose get_current_position() override;
  static void wait_connect_success(ros::Publisher publisher);
};