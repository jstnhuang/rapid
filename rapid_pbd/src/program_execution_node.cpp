#include "rapid_pbd/action_names.h"
#include "rapid_pbd/program_executor.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "program_executor");
  ros::NodeHandle nh;
  ros::Publisher is_running_pub =
      nh.advertise<std_msgs::Bool>("is_running", 5, true);
  rapid::pbd::ProgramExecutionServer server(rapid::pbd::kProgramActionName,
                                            is_running_pub);
  server.Start();
  ros::spin();
  return 0;
}
