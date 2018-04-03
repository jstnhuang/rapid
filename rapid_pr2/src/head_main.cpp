#include <signal.h>
#include <stdlib.h>
#include <iostream>

#include "rapid_pr2/head.h"
#include "rapid_robot/joint_state_reader.h"
#include "rapid_ros/time.h"
#include "rapid_utils/deg_rad.h"

sig_atomic_t volatile g_request_shutdown = 0;

void PrintUsage() {
  std::cout << "Usage: rosrun rapid_pr2 head PAN_DEGREES TILT_DEGREES"
            << std::endl;
  std::cout << "Pan: -168 to 168, positive looks left" << std::endl;
  std::cout << "Tilt: -30 to 60, positive looks down" << std::endl;
}

void HandleSigint(int signum) { g_request_shutdown = 1; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pr2_head_main",
            ros::init_options::NoSigintHandler);
  if (argc < 3) {
    PrintUsage();
    return 0;
  }

  signal(SIGINT, HandleSigint);
  ros::NodeHandle nh;
  rapid::WaitForTime();

  rapid::JointStateReader js_reader;
  js_reader.Start();
  rapid::pr2::Head head(js_reader);

  double pan_degrees = atof(argv[1]);
  double tilt_degrees = atof(argv[2]);
  bool success = head.StartPanTilt(rapid::Degrees(pan_degrees),
                                   rapid::Degrees(tilt_degrees));
  if (!success) {
    ROS_ERROR("Failed to send torso goal.");
    return 1;
  }
  while (!g_request_shutdown) {
    if (head.IsDone()) {
      break;
    }
    ros::spinOnce();
  }

  if (!head.IsDone()) {
    head.Cancel();
  }
  ros::shutdown();

  return 0;
}
