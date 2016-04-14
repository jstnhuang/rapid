#include "rapid_manipulation/gripper.h"

#include "actionlib/client/simple_action_client.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

using actionlib::SimpleClientGoalState;

namespace rapid {
namespace manipulation {
const std::string Gripper::LEFT_GRIPPER_ACTION =
    "l_gripper_controller/gripper_action";
const std::string Gripper::RIGHT_GRIPPER_ACTION =
    "r_gripper_controller/gripper_action";
const double Gripper::OPEN_THRESHOLD = 0.005;
const double Gripper::OPEN = 0.09;
const double Gripper::CLOSED = 0.00;

Gripper::Gripper(const int gripper_id, GripperClient* client)
    : gripper_id_(gripper_id), gripper_client_(client) {
  std::string action_name;
  if (gripper_id == Gripper::LEFT_GRIPPER) {
    action_name = LEFT_GRIPPER_ACTION;
  } else if (gripper_id == Gripper::RIGHT_GRIPPER) {
    action_name = RIGHT_GRIPPER_ACTION;
  } else {
    ROS_ERROR("Bad gripper ID: %d", gripper_id);
    action_name = RIGHT_GRIPPER_ACTION;
  }
}

Gripper::~Gripper() { delete gripper_client_; }

bool Gripper::SetPosition(double position, double effort) const {
  if (position > Gripper::OPEN) {
    ROS_WARN("Clamping gripper position %0.3f to max: %0.3f", position,
             Gripper::OPEN);
    position = Gripper::OPEN;
  } else if (position < Gripper::CLOSED) {
    ROS_WARN("Clamping gripper position %0.3f to min: %0.3f", position,
             Gripper::CLOSED);
    position = Gripper::CLOSED;
  }
  if (!gripper_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR("The gripper action server was not available after 5 seconds!");
    return false;
  }

  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.position = position;
  goal.command.max_effort = effort;

  gripper_client_->sendGoal(goal);
  if (!gripper_client_->waitForResult(ros::Duration(10))) {
    ROS_ERROR(
        "The gripper action server did not return a result after 10 seconds!");
    return false;
  }
  SimpleClientGoalState state = gripper_client_->getState();
  if (state == SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    ROS_ERROR("Gripper goal state: %s\n", state.toString().c_str());
    return false;
  }
}

double Gripper::GetPosition() const {
  tf::StampedTransform transform;
  std::string destination_frame;
  std::string original_frame;

  // set frames based on whether this is a right or left gripper
  if (gripper_id_ == Gripper::LEFT_GRIPPER) {
    destination_frame = "l_gripper_l_finger_tip_link";
    original_frame = "l_gripper_r_finger_tip_link";
  } else if (gripper_id_ == Gripper::RIGHT_GRIPPER) {
    destination_frame = "r_gripper_l_finger_tip_link";
    original_frame = "r_gripper_r_finger_tip_link";
  }

  // get the transform between the fingertips
  try {
    transform_listener_.waitForTransform(original_frame, destination_frame,
                                         ros::Time(0), ros::Duration(10.0));
    transform_listener_.lookupTransform(original_frame, destination_frame,
                                        ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  // subtract small positive offset so that 0 means closed
  double gripper_offset = transform.getOrigin().y();
  ROS_INFO("gripper_offset = %f", gripper_offset);
  return gripper_offset - 0.032;
}

bool Gripper::IsOpen() const { return GetPosition() > Gripper::OPEN_THRESHOLD; }

bool Gripper::Open(double effort) const {
  return Gripper::SetPosition(Gripper::OPEN, effort);
}

bool Gripper::Close(double effort) const {
  return Gripper::SetPosition(Gripper::CLOSED, effort);
}
}  // namespace manipulation
}  // namespace rapid
