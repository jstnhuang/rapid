#include <iostream>
#include <map>
#include <string>

#include "rapid_robot/joint_state_reader.h"
#include "rapid_ros/time.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_robot_joint_state_reader");
  ros::NodeHandle nh;
  rapid::WaitForTime();
  rapid::JointStateReader reader;
  reader.Start();
  bool success = reader.WaitForMessages(ros::Duration(5));
  if (!success) {
    ROS_ERROR("Unable to read from /joint_states topic after 5 seconds!");
    return 1;
  }

  if (argc == 1) {
    std::cout << "Alternate usage: rosrun rapid_robot joint_state_reader "
                 "torso_lift_link LINK2 LINK3 ..."
              << std::endl;
    std::map<std::string, double> joint_values = reader.positions();
    for (std::map<std::string, double>::const_iterator it =
             joint_values.begin();
         it != joint_values.end(); ++it) {
      std::cout << it->first << "\t" << it->second << std::endl;
    }
  } else {
    for (int argi = 1; argi < argc; ++argi) {
      std::string name(argv[argi]);
      reader.WaitForJoint(name, ros::Duration(1));
      if (!reader.HasJoint(name)) {
        std::cout << name << " not found!" << std::endl;
      } else {
        std::cout << name << "\t" << reader.position(name) << std::endl;
      }
    }
  }

  return 0;
}
