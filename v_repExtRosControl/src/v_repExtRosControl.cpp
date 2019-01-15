#include "v_repExtRosControl/v_repExtRosControl.h"
#include "hardware_interface_vrep/robot.h"
#include "v_repPlusPlus/Plugin.h"
#include "stubs.h"
#include <ostream>

std::vector<hardware_interface_vrep::Robot *> robots_list;

void createRobot(SScriptCallBack *p, const char *cmd, createRobot_in *in, createRobot_out *out) {
  robots_list.push_back(new hardware_interface_vrep::Robot(ros::NodeHandle(in->robot_name)));
  out->handle = robots_list.size();   // robot handle is position in robot_list +1 !
}

void addJoint(SScriptCallBack *p, const char *cmd, addJoint_in *in, addJoint_out *out) {
  // Must be at least one robot created!
  if (robots_list.size() == 0) {
    out->result = -1;
    return;
  }

  // Robot handle must be correct
  if (robots_list.size() > in->robot_handle) {
    out->result = -1;
    return;
  }

  // Joint must be valid and object type
  if (simIsHandleValid(in->joint_handle, sim_appobj_object_type) != 1) {
    out->result = -1;
    return;
  }

  // TODO Check if joint is already added to any robot

  // Try to add joint, and addJoint must return True
  if (!robots_list[in->robot_handle - 1]->addJoint(in->joint_handle, in->joint_name, in->max_force) ) {
    out->result = -1;
    return;
  }

  // success return 0
  out->result = 0;
}

void initRobot(SScriptCallBack *p, const char *cmd, initRobot_in *in, initRobot_out *out) {
  if (robots_list.size() == 0) {
    out->result = -1;
    return;
  }

  // Robot handle must be correct
  if (robots_list.size() > in->robot_handle) {
    out->result = -1;
    return;
  }

  robots_list[in->robot_handle - 1]->init();
  out->result = 0;
}

class RosControlPlugin : public vrep::Plugin {
  private:
    // ros::AsyncSpinner *spinner;
  public:

    void onStart() {
      if (!registerScriptStuff()) throw std::runtime_error("NI script stuff initialization failed");

      simSetModuleInfo(PLUGIN_NAME, 0, "RosControl", 0);
      simSetModuleInfo(PLUGIN_NAME, 1, BUILD_DATE, 0);
    }

    void onSimulationAboutToEnd() {
      std::cout << "Clearing robots" << '\n';

      for (auto it = robots_list.begin(); it < robots_list.end(); it++) {
        delete (*it);
      }

      robots_list.clear();
    }

    void onModuleHandle(char *name) {
      if ((name == NULL) || (std::string(PLUGIN_NAME).compare(name) == 0) ) {
        for (auto it = robots_list.begin(); it < robots_list.end(); it++) {
          (*it)->update();
        }
      }
    }
};

VREP_PLUGIN(PLUGIN_NAME, 1, RosControlPlugin)
