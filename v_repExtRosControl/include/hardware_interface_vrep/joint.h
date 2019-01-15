#ifndef JOINT_H
#define JOINT_H

#include "v_repLib.h"

#define HALF_LIMIT_PERCENT 0.3
#define NEGATIVE_LIMIT     ((1.0 - HALF_LIMIT_PERCENT) * 2 * M_PI)
#define POSITIVE_LIMIT     ((HALF_LIMIT_PERCENT) * 2 * M_PI)


namespace hardware_interface_vrep {
enum ControllerType {
  None,
  Position,
  Velocity,
  Effort
};


class Joint {
  public:
    Joint() {
      position         = 0;
      velocity         = 0;
      effort           = 0;
      position_command = 0;
      velocity_command = 0;
      effort_command   = 0;
      controller_type  = None;
      prepareSwitch(None);
    };


    virtual void update() = 0;

    virtual void setCommand() = 0;

    void prepareSwitch(ControllerType new_controller_type) {
      controller_to_switch_ = new_controller_type;
    }

    void switchController() {
      controller_type = controller_to_switch_;
    }

    double position;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;

    ControllerType controller_type; // current controller type


  protected:
    ControllerType controller_to_switch_; // controller to switch in switchController method
};

class JointVrep : public Joint {
  public:
    int handle; // Vrep object handle
    int type;   // sim_joint_prismatic_subtype or sim_joint_revolute_subtype
    float max_force;

    JointVrep(int joint_handle = 0) : Joint(), handle(joint_handle) {
      type = simGetJointType(handle);
      max_force = 0;
      previous_position_ = 0.0;
      rotations_         = 0;
    };

    JointVrep(const JointVrep &joint) : JointVrep(joint.handle) {
      max_force = joint.max_force;
    };

    void update() {
      // Update position
      float pos;

      simGetJointPosition(handle, &pos);

      if (type == sim_joint_revolute_subtype) {
        if (pos < 0) {
          pos += 2 * M_PI;
        }

        if ( (pos < POSITIVE_LIMIT) && (previous_position_ > NEGATIVE_LIMIT) ) {
          rotations_++;
        }

        if ( (pos > NEGATIVE_LIMIT) && (previous_position_ < POSITIVE_LIMIT)) {
          rotations_--;
        }

        previous_position_ = pos;
        position = (double)pos + 2 * M_PI * rotations_;
      } else if (type == sim_joint_prismatic_subtype) position = pos;

      // Update velocity
      float vel;
      simGetObjectFloatParameter(handle, sim_jointfloatparam_velocity, &vel);
      velocity = vel;

      // Update torque
      float torque;
      simGetJointForce(handle, &torque);
      effort = -torque;
    }

    void setCommand() {
      switch (controller_type) {
        case None:
          break;

        case Position: {
          simSetJointTargetPosition(handle, position_command);
          break;
        }

        case Velocity: {
          simSetJointTargetVelocity(handle, velocity_command);
          break;
        }

        case Effort: {
          float command = effort_command;

          if (command > 0) {
            simSetJointTargetVelocity(handle, 99999.0);
          } else if (command < 0) {
            simSetJointTargetVelocity(handle, -99999.0);
            command *= -1;
          } else {
            simSetJointTargetVelocity(handle, 0);
          }

          simSetJointForce(handle, command);
          break;
        }

        default:
          break;
      }
    }

    void switchController() {
      Joint::switchController();

      if (controller_type == Position) {
        simSetJointTargetVelocity(handle, 0);
        simSetJointForce(handle, max_force);
        simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 1);      // Enable control loop
        simSetObjectInt32Parameter(handle, sim_jointintparam_motor_enabled, 1);     // Enable Motor
      } else if (controller_type == Velocity) {
        simSetJointTargetVelocity(handle, 0);
        simSetJointForce(handle, max_force);
        simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 0);      // Disable control loop
        simSetObjectInt32Parameter(handle, sim_jointintparam_motor_enabled, 1);     // Enable Motor
      } else if (controller_type == Effort) {
        simSetJointTargetVelocity(handle, 0);
        simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 0);      // Disable control loop
        simSetObjectInt32Parameter(handle, sim_jointintparam_motor_enabled, 1);     // Enable Motor
      } else {
        simSetJointTargetVelocity(handle, 0.0);
        simSetJointForce(handle, max_force);
        simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 0);      // Disable control loop
        simSetObjectInt32Parameter(handle, sim_jointintparam_motor_enabled, 0);     // Disable Motor
      }
    }

  private:
    float previous_position_;
    int rotations_;
};
}

#endif /* end of include guard: JOINT_H */
