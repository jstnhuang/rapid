#include "rapid_pbd/action_names.h"
#include "rapid_pbd/program_executor.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pbd_program_executor_node");
  rapid::pbd::ProgramExecutionServer server(rapid::pbd::kProgramActionName);
  server.Start();
  ros::spin();
  return 0;
}
