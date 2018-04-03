#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "rapid_pr2/gripper.h"
#include "rapid_ros/time.h"
#include "ros/ros.h"

sig_atomic_t volatile g_request_shutdown = 0;

void PrintUsage() {
  std::cout << "Usage: rosrun rapid_pr2 pr2_gripper_demo open left"
            << std::endl;
  std::cout << "       rosrun rapid_pr2 pr2_gripper_demo close right"
            << std::endl;
  std::cout << "Usage (with max force): rosrun rapid_pr2 pr2_gripper_demo "
               "close right "
               "50"
            << std::endl;
}

void HandleSigint(int signum) { g_request_shutdown = 1; }

void WaitFor(rapid::pr2::Gripper& gripper) {
  while (!g_request_shutdown) {
    if (gripper.IsDone()) {
      break;
    }
    ros::spinOnce();
  }
  if (!gripper.IsDone()) {
    gripper.Cancel();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pr2_gripper_main",
            ros::init_options::NoSigintHandler);
  if (argc < 3) {
    PrintUsage();
    return 0;
  }

  signal(SIGINT, HandleSigint);
  ros::NodeHandle nh;
  rapid::WaitForTime();

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
