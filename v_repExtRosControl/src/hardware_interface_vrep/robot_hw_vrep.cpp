#include <hardware_interface_vrep/robot_hw_vrep.h>

namespace hardware_interface_vrep {
RobotHWVrep::RobotHWVrep() : num_joints_(0) {
}

RobotHWVrep::~RobotHWVrep() {
}

void RobotHWVrep::addJoint(JointVrep &joint, std::string name) {
  joints_.insert(std::pair<std::string, JointVrep>(name, joint));
  num_joints_++;
};

bool RobotHWVrep::init(ros::NodeHandle &nh) {
  std::map<std::string, JointVrep>::iterator iter;

  for (iter = joints_.begin(); iter != joints_.end(); iter++) {
    iter->second.controller_type = None;
    hardware_interface::JointStateHandle js_handle(iter->first, &iter->second.position,
                                                   &iter->second.velocity,
                                                   &iter->second.effort);

    hardware_interface::JointHandle pj_handle(js_handle, &iter->second.position_command);
    hardware_interface::JointHandle vj_handle(js_handle, &iter->second.velocity_command);
    hardware_interface::JointHandle ej_handle(js_handle, &iter->second.effort_command);

    joint_state_interface_.registerHandle(js_handle);
    position_joint_interface_.registerHandle(pj_handle);
    velocity_joint_interface_.registerHandle(vj_handle);
    effort_joint_interface_.registerHandle(ej_handle);

    joint_limits_interface::JointLimits limits;

    if (getJointLimits(iter->first, nh, limits)) {
      try {
        joint_limits_interface::PositionJointSaturationHandle pjs_handle(pj_handle, limits);
        position_joint_saturation_interface_.registerHandle(pjs_handle);
      } catch (joint_limits_interface::JointLimitsInterfaceException &e) {
        ROS_WARN_STREAM(e.what());
      }
      try {
        joint_limits_interface::VelocityJointSaturationHandle vjs_handle(vj_handle, limits);
        velocity_joint_saturation_interface_.registerHandle(vjs_handle);
      } catch (joint_limits_interface::JointLimitsInterfaceException &e) {
        ROS_WARN_STREAM(e.what());
      }
      try {
        joint_limits_interface::EffortJointSaturationHandle ejs_handle(ej_handle, limits);
        effort_joint_saturation_interface_.registerHandle(ejs_handle);
      } catch (joint_limits_interface::JointLimitsInterfaceException &e) {
        ROS_WARN_STREAM(e.what());
      }
    }
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&effort_joint_interface_);

  return true;
}

void RobotHWVrep::read(const ros::Time& time, const ros::Duration& period) {
  std::map<std::string, JointVrep>::iterator iter;

  for (iter = joints_.begin(); iter != joints_.end(); iter++) {
    iter->second.update();
  }
}

void RobotHWVrep::write(const ros::Time& time, const ros::Duration& period) {
  std::map<std::string, JointVrep>::iterator iter;

  position_joint_saturation_interface_.enforceLimits(period);
  velocity_joint_saturation_interface_.enforceLimits(period);
  effort_joint_saturation_interface_.enforceLimits(period);

  for (iter = joints_.begin(); iter != joints_.end(); iter++) {
    iter->second.setCommand();
  }
}

bool RobotHWVrep::prepareSwitch(const std::list<hardware_interface::ControllerInfo>&  start_list,
                                const std::list<hardware_interface::ControllerInfo> & stop_list) {
  std::ostringstream ss;

  ss << "Preparing controllers switch:\n";

  if (stop_list.size()) {
    ss << "stop controllers:\n";
    std::list<hardware_interface::ControllerInfo>::const_iterator controller_info;

    for (controller_info = stop_list.begin(); controller_info != stop_list.end(); controller_info++) {
      ss << "\tName: " << controller_info->name << "\n";
      ss << "\t-type: " << controller_info->type << "\n";
      ss << "\t-claimed resources:\n";
      std::vector<hardware_interface::InterfaceResources>::const_iterator claimed_resource;

      for (claimed_resource = controller_info->claimed_resources.begin();
           claimed_resource != controller_info->claimed_resources.end();
           claimed_resource++) {
        std::string interface = claimed_resource->hardware_interface;
        ss << "\t  -interface type: " << interface << "\n";

        std::set<std::string>::const_iterator resource;

        for (resource = claimed_resource->resources.begin(); resource != claimed_resource->resources.end();
             resource++) {
          ss << "\t    -resource name: " << *resource << "\t";
          joints_[*resource].prepareSwitch(None);
        }
      }
    }
  }

  if (start_list.size()) {
    ss << "start controllers:\n";
    std::list<hardware_interface::ControllerInfo>::const_iterator controller_info;

    for (controller_info = start_list.begin(); controller_info != start_list.end(); controller_info++) {
      ss << "\tName: " << controller_info->name << "\n";
      ss << "\t-type: " << controller_info->type << "\n";
      ss << "\t-claimed resources:\n";
      std::vector<hardware_interface::InterfaceResources>::const_iterator claimed_resource;

      for (claimed_resource = controller_info->claimed_resources.begin();
           claimed_resource != controller_info->claimed_resources.end();
           claimed_resource++) {
        std::string interface = claimed_resource->hardware_interface;
        ss << "\t  -interface type: " << interface << "\n";

        std::set<std::string>::const_iterator resource;

        for (resource = claimed_resource->resources.begin(); resource != claimed_resource->resources.end();
             resource++) {
          ss << "\t    -resource name: " << *resource << "\t";

          if (!interface.compare("hardware_interface::EffortJointInterface")) {
            joints_[*resource].prepareSwitch(Effort);
          } else if (!interface.compare("hardware_interface::PositionJointInterface")) {
            joints_[*resource].prepareSwitch(Position);
          } else if (!interface.compare("hardware_interface::VelocityJointInterface")) {
            joints_[*resource].prepareSwitch(Velocity);
          } else {
            joints_[*resource].prepareSwitch(None);
          }
        }
      }
    }
  }

  ROS_INFO_STREAM(ss.str());
  return true;
}

void RobotHWVrep::doSwitch(const std::list<hardware_interface::ControllerInfo> &                      start_list,
                           const std::                     list<hardware_interface::ControllerInfo> & stop_list) {
  std::map<std::string, JointVrep>::iterator iter;

  for (iter = joints_.begin(); iter != joints_.end(); iter++) {
    iter->second.switchController();
  }
}
}
