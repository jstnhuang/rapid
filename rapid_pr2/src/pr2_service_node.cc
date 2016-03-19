#include "ros/ros.h"

#include "boost/shared_ptr.hpp"
#include "rapid_msgs/UseGrippers.h"
#include "rapid_msgs/MoveArm.h"

#include "rapid_pr2/pr2.h"

using boost::shared_ptr;
using rapid::pr2::Pr2;

class GripperService {
 public:
  GripperService(shared_ptr<Pr2> pr2)
      : pr2_(pr2),
        nh_(),
        server_(nh_.advertiseService("use_grippers", &GripperService::Callback,
                                     this)) {}
  bool Callback(rapid_msgs::UseGrippersRequest& request,
                rapid_msgs::UseGrippersResponse& response) {
    bool left_success = false;
    if (request.left_action == request.OPEN) {
      left_success = pr2_->left_gripper.Open();
    } else if (request.left_action == request.CLOSE) {
      left_success = pr2_->left_gripper.Close();
    }

    bool right_success = false;
    if (request.right_action == request.OPEN) {
      right_success = pr2_->right_gripper.Open();
    } else if (request.right_action == request.CLOSE) {
      right_success = pr2_->right_gripper.Close();
    }
    return left_success && right_success;
  }

 private:
  shared_ptr<Pr2> pr2_;
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
};

class MoveArmService {
 public:
  MoveArmService(shared_ptr<Pr2> pr2)
      : pr2_(pr2),
        nh_(),
        server_(nh_.advertiseService("move_arm", &MoveArmService::Callback,
                                     this)) {}
  bool Callback(rapid_msgs::MoveArmRequest& request,
                rapid_msgs::MoveArmResponse& response) {
    if (request.arm_id == request.LEFT) {
      return pr2_->left_arm.MoveToPoseGoal(request.goal);
    } else if (request.arm_id == request.RIGHT) {
      return pr2_->right_arm.MoveToPoseGoal(request.goal);
    } else {
      return false;
    }
  }

 private:
  shared_ptr<Pr2> pr2_;
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_services");
  shared_ptr<Pr2> pr2 = rapid::pr2::BuildReal();
  GripperService gripper_service(pr2);
  MoveArmService move_arm_service(pr2);
  ros::spin();
  return 0;
}
