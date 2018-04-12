#include "rapid_manipulation/moveit_error_code.h"

#include "moveit_msgs/MoveItErrorCodes.h"

namespace rapid {
bool IsSuccess(const moveit::planning_interface::MoveItErrorCode& error_code) {
  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

bool IsSuccess(const moveit_msgs::MoveItErrorCodes& error_code) {
  moveit::planning_interface::MoveItErrorCode code(error_code);
  return IsSuccess(code);
}

std::string ErrorString(
    const moveit::planning_interface::MoveItErrorCode& error_code) {
  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    return "SUCCESS";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::FAILURE) {
    return "FAILURE";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED) {
    return "PLANNING_FAILED";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) {
    return "INVALID_MOTION_PLAN";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::
                 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE) {
    return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED) {
    return "CONTROL_FAILED";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA) {
    return "UNABLE_TO_AQUIRE_SENSOR_DATA";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT) {
    return "TIMED_OUT";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
    return "PREEMPTED";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION) {
    return "START_STATE_IN_COLLISION";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::
                                   START_STATE_VIOLATES_PATH_CONSTRAINTS) {
    return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION) {
    return "GOAL_IN_COLLISION";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS) {
    return "GOAL_VIOLATES_PATH_CONSTRAINTS";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED) {
    return "GOAL_CONSTRAINTS_VIOLATED";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME) {
    return "INVALID_GROUP_NAME";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS) {
    return "INVALID_GOAL_CONSTRAINTS";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE) {
    return "INVALID_ROBOT_STATE";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME) {
    return "INVALID_LINK_NAME";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME) {
    return "INVALID_OBJECT_NAME";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE) {
    return "FRAME_TRANSFORM_FAILURE";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE) {
    return "COLLISION_CHECKING_UNAVAILABLE";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE) {
    return "ROBOT_STATE_STALE";
  } else if (error_code.val ==
             moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE) {
    return "SENSOR_INFO_STALE";
  } else if (error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION) {
    return "NO_IK_SOLUTION";
  } else {
    return "Unknown error";
  }
}

std::string ErrorString(const moveit_msgs::MoveItErrorCodes& error_code) {
  moveit::planning_interface::MoveItErrorCode code(error_code);
  return ErrorString(code);
}
}  // namespace rapid
