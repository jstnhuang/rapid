#include <stdlib.h>
#include <iostream>

#include "rapid_pr2/torso.h"
#include "rapid_robot/joint_state_reader.h"
#include "rapid_ros/time.h"

void PrintUsage() {
  std::cout << "Usage: rosrun rapid_pr2 torso 0.31" << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    PrintUsage();
    return 0;
  }

  ros::init(argc, argv, "rapid_pr2_torso_main");
  ros::NodeHandle nh;
  rapid::WaitForTime();

  rapid::JointStateReader js_reader;
  js_reader.Start();
  rapid::pr2::Torso torso(js_reader);

  double height = atof(argv[1]);
  bool success = torso.StartMoving(height);
  if (!success) {
    ROS_ERROR("Failed to send torso goal.");
    return 1;
  }
  while (!torso.IsDone() && ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
