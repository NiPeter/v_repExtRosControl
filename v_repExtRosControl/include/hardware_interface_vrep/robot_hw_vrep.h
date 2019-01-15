#ifndef ROBOT_HW_VREP_H
#define ROBOT_HW_VREP_H

#include <ros/ros.h>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <hardware_interface_vrep/joint.h>

#include "v_repLib.h"

namespace hardware_interface_vrep {
class RobotHWVrep : public hardware_interface::RobotHW {
  public:
    RobotHWVrep();
    ~RobotHWVrep();
    void addJoint(JointVrep & joint, std::string name);
    bool init(ros::NodeHandle &nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);
    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>&  start_list,
                       const std::list<hardware_interface::ControllerInfo> & stop_list);
    void doSwitch(const std:: list<hardware_interface::ControllerInfo>& /*start_list*/,
                  const std:: list<hardware_interface::ControllerInfo>& /*stop_list*/);


  protected:
    hardware_interface::JointStateInterface joint_state_interface_;

    hardware_interface::PositionJointInterface position_joint_interface_;
    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;

    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;

    hardware_interface::EffortJointInterface effort_joint_interface_;
    joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;


    unsigned int num_joints_;
    std::map<std::string, JointVrep> joints_;
};
}

#endif /* end of include guard: ROBOT_HW_VREP_H */
