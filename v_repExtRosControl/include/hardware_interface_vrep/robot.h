#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <hardware_interface_vrep/robot_hw_vrep.h>
#include <controller_manager/controller_manager.h>


namespace hardware_interface_vrep {
class Robot {
  public:
    Robot(ros::NodeHandle robot_nh);
    ~Robot();
    void init();
    void update();
    bool addJoint(int joint_handle, std::string name, float max_force = 2.5);

  private:
    ros::NodeHandle robot_nh_;
    ros::AsyncSpinner *spinner_;
    ros::CallbackQueue callback_queue;
    controller_manager::ControllerManager *controller_manager_;

    RobotHWVrep robot_hardware_;
    ros::Time previous_time_;
};
}


#endif /* end of include guard: ROBOT_H */
