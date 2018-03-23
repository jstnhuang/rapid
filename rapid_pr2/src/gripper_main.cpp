#include <stdlib.h>
#include <string>

#include "rapid_pr2/gripper.h"
#include "ros/ros.h"

void PrintUsage() {
  ROS_INFO("Usage: rosrun rapid_pr2 pr2_gripper_demo open left");
  ROS_INFO("       rosrun rapid_pr2 pr2_gripper_demo close right");
  ROS_INFO(
      "Usage (with max force): rosrun rapid_pr2 pr2_gripper_demo close right "
      "50");
}

void WaitFor(const rapid::pr2::Gripper& gripper) {
  while (ros::ok() && gripper.IsDone()) {
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pr2_gripper_main");
  if (argc < 3) {
    PrintUsage();
    return 0;
  }

  std::string command(argv[1]);
  std::string side(argv[2]);

  rapid::pr2::Gripper left = rapid::pr2::Gripper::Left();
  rapid::pr2::Gripper right = rapid::pr2::Gripper::Right();

  if (command == "open") {
    if (side == "left") {
      left.StartOpening();
      WaitFor(left);
    } else if (side == "right") {
      right.StartOpening();
      WaitFor(right);
    } else {
      PrintUsage();
      return 0;
    }
  } else if (command == "close") {
    double max_effort = -1;
    if (argc >= 4) {
      max_effort = atof(argv[3]);
    }
    if (side == "left") {
      if (max_effort > 0) {
        left.StartClosing(max_effort);
      } else {
        left.StartClosing();
      }
      WaitFor(left);
    } else if (side == "right") {
      if (max_effort > 0) {
        right.StartClosing(max_effort);
      } else {
        right.StartClosing();
      }
      WaitFor(right);
    } else {
      PrintUsage();
      return 0;
    }
  } else {
    PrintUsage();
    return 0;
  }

  return 0;
}
