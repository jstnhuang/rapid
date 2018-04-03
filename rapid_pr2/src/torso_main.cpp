#include <signal.h>
#include <stdlib.h>
#include <iostream>

#include "rapid_pr2/torso.h"
#include "rapid_robot/joint_state_reader.h"
#include "rapid_ros/time.h"

sig_atomic_t volatile g_request_shutdown = 0;

void PrintUsage() {
  std::cout << "Usage: rosrun rapid_pr2 torso 0.31" << std::endl;
}

void HandleSigint(int signum) { g_request_shutdown = 1; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pr2_torso_main",
            ros::init_options::NoSigintHandler);
  if (argc < 2) {
    PrintUsage();
    return 0;
  }
  signal(SIGINT, HandleSigint);
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
  while (!g_request_shutdown) {
    if (torso.IsDone()) {
      break;
    }
    ros::spinOnce();
  }

  if (!torso.IsDone()) {
    torso.Cancel();
  }
  ros::shutdown();

  return 0;
}
