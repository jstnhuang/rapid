#ifndef _RAPID_PBD_PROGRAM_EXECUTOR_H_
#define _RAPID_PBD_PROGRAM_EXECUTOR_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "rapid_pbd_msgs/Program.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
class ProgramExecutionServer {
 public:
  ProgramExecutionServer(const std::string& action_name,
                         const ros::Publisher& is_running_pub,
                         ActionClients* action_clients,
                         const RobotConfig& robot_config);
  void Start();

 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rapid_pbd_msgs::ExecuteProgramAction> server_;
  ros::ServiceClient freeze_arm_client_;
  ros::Publisher is_running_pub_;
  ActionClients* action_clients_;
  const RobotConfig& robot_config_;

  void Execute(const rapid_pbd_msgs::ExecuteProgramGoalConstPtr& goal);
  static bool IsValid(const rapid_pbd_msgs::Program& program);
  void PublishIsRunning(bool is_running);
  void Cancel(const std::string& error);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PROGRAM_EXECUTOR_H_
