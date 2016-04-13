// Implements a mock Blinky actionlib server, which we test against when using
// the Blinky class.

#include "actionlib/server/simple_action_server.h"
#include "blinky/FaceAction.h"

using actionlib::SimpleActionServer;
using blinky::FaceAction;
using blinky::FaceGoal;
using blinky::FaceGoalConstPtr;
using blinky::FaceResult;

namespace rapid {
namespace display {
class MockBlinkyServer {
 public:
  MockBlinkyServer()
      : nh_(),
        server_(nh_, "blinky",
                boost::bind(&MockBlinkyServer::ExecuteCb, this, _1), false),
        last_goal_() {}
  void Start() { server_.start(); }

  // Processes all available callbacks, and gets the most recent goal that was
  // processed. If you are testing with the MockBlinkyServer, you are guaranteed
  // that the execute callback was processed after calling this method.
  //
  // See roscpp documentation on callbacks and spinning.
  FaceGoal WaitForGoal() {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    return last_goal_;
  }

 protected:
  void ExecuteCb(const FaceGoalConstPtr& goal) {
    last_goal_ = *goal;
    FaceResult result;
    if (goal->display_type == FaceGoal::ASK_CHOICE) {
      result.choice = goal->choices[0];
    }
    server_.setSucceeded(result);
  }
  ros::NodeHandle nh_;
  SimpleActionServer<FaceAction> server_;
  FaceGoal last_goal_;
};
}  // namespace display
}  // namespace rapid
