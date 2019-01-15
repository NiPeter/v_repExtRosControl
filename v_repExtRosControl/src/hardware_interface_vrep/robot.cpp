#include <hardware_interface_vrep/robot.h>

namespace hardware_interface_vrep {
/*
 *  Constructor
 */
Robot::Robot(ros::NodeHandle robot_nh) : robot_nh_(robot_nh) {
  robot_nh_.setCallbackQueue(&callback_queue);

  spinner_ = new ros::AsyncSpinner(2, &callback_queue);
  spinner_->start();

  controller_manager_ = new controller_manager::ControllerManager(&robot_hardware_, robot_nh_);
};

/*
 *  Destructor
 */
Robot::~Robot() {
  delete controller_manager_;
  controller_manager_ = NULL;

  delete spinner_;
  spinner_ = NULL;
}

/*
 *  init hardware
 */
void Robot::init() {
  robot_hardware_.init(robot_nh_);
  previous_time_ = ros::Time::now();
}

/*
 *  Update once
 */
void Robot::update() {
  if (ros::ok() && spinner_ != NULL) {
    ros::Time current_time = ros::Time::now();
    ros::Duration period   = ros::Duration(current_time - previous_time_);
    previous_time_ = current_time;

    robot_hardware_.read(current_time, period);
    controller_manager_->update(current_time, period, false);
    robot_hardware_.write(current_time, period);
    // callback_queue.callAvailable(ros::WallDuration());
  }
}

bool Robot::addJoint(int joint_handle, std::string name, float max_force) {
  // Joint must be prismatic or revolute
  int joint_type = simGetJointType(joint_handle);

  if ( (joint_type == -1) || (joint_type == sim_joint_spherical_subtype)) {
    return false;
  }

  JointVrep joint(joint_handle);
  joint.max_force = max_force;
  robot_hardware_.addJoint(joint, name);

  return true;
}
}
