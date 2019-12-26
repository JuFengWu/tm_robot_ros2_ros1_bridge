#include "sensor_msgs/JointState.h"


class ModleInterface{
public:
  virtual void set_model_initial_pose(sensor_msgs::JointState& msg,double allInitialPose =0)=0;
  virtual void set_model_name(sensor_msgs::JointState& msg)=0;    
};

class TmGraspModel : public ModleInterface
{

public:
  void set_model_initial_pose(sensor_msgs::JointState& msg,double allInitialPose =0) override;
  void set_model_name(sensor_msgs::JointState& msg) override;
};