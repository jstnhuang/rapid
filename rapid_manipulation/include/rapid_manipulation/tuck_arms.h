// PR2-specific class for deploying/tucking arms.
#ifndef _RAPID_MANIPULATION_TUCK_ARMS_H_
#define _RAPID_MANIPULATION_TUCK_ARMS_H_

#include "actionlib/client/simple_action_client.h"
#include "gmock/gmock.h"
#include "pr2_common_action_msgs/TuckArmsAction.h"

#include "rapid_ros/action_client.h"

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
  explicit Pr2TuckArms(rapid_ros::ActionClientInterface<
      pr2_common_action_msgs::TuckArmsAction>* client);
  bool TuckArms();
  bool DeployLeft();
  bool DeployRight();
  bool DeployArms();

  void set_server_wait_time(double server_wait_time);

 private:
  bool ExecuteAction(bool tuck_left, bool tuck_right);
  rapid_ros::ActionClientInterface<pr2_common_action_msgs::TuckArmsAction>*
      client_;
  double server_wait_time_;  // Wait time for tuck arms server, in seconds.
};

class MockTuckArms : public TuckArmsInterface {
 public:
  MOCK_METHOD0(TuckArms, bool());
  MOCK_METHOD0(DeployLeft, bool());
  MOCK_METHOD0(DeployRight, bool());
  MOCK_METHOD0(DeployArms, bool());
};
}  //  namespace manipulation
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_TUCK_ARMS_H_
