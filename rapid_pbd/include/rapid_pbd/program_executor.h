#ifndef _RAPID_PBD_PROGRAM_EXECUTOR_H_
#define _RAPID_PBD_PROGRAM_EXECUTOR_H_

#include <string>

#include "actionlib/server/simple_action_server.h"

#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "rapid_pbd_msgs/Program.h"

namespace rapid {
namespace pbd {
class ProgramExecutionServer {
 public:
  ProgramExecutionServer(const std::string& action_name);
  void Start();

 private:
  actionlib::SimpleActionServer<rapid_pbd_msgs::ExecuteProgramAction> server_;

  void Execute(const rapid_pbd_msgs::ExecuteProgramGoalConstPtr& goal);
  static bool IsValid(const rapid_pbd_msgs::Program& program);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PROGRAM_EXECUTOR_H_
