// PR2-specific class for deploying/tucking arms.
#ifndef _RAPID_MANIPULATION_TUCK_ARMS_H_
#define _RAPID_MANIPULATION_TUCK_ARMS_H_

#include "actionlib/client/simple_action_client.h"
#include "pr2_common_action_msgs/TuckArmsAction.h"

namespace rapid {
namespace manipulation {
class TuckArmsInterface {
 public:
  virtual ~TuckArmsInterface() {}
  virtual bool TuckArms() = 0;     // Tuck both
  virtual bool DeployLeft() = 0;   // Deploy left, tuck right
  virtual bool DeployRight() = 0;  // Deploy right, tuck left
  virtual bool DeployArms() = 0;   // Deploy both
};

class Pr2TuckArms : public TuckArmsInterface {
 public:
  Pr2TuckArms();
  bool TuckArms();
  bool DeployLeft();
  bool DeployRight();
  bool DeployArms();

 private:
  bool ExecuteAction(bool tuck_left, bool tuck_right);
  actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> client_;
  double server_wait_time_;  // Wait time for tuck arms server, in seconds.
};
}  //  namespace manipulation
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_TUCK_ARMS_H_
