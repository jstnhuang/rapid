#include "rapid_pbd/action_executor.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "ros/ros.h"

#include "rapid_pbd/action_names.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using control_msgs::FollowJointTrajectoryAction;
using rapid_pbd_msgs::Action;

namespace rapid {
namespace pbd {
ActionExecutor::ActionExecutor(const Action& action)
    : action_(action),
      gripper_client_(kGripperActionName, true),
      l_gripper_client_(kLeftGripperActionName, true),
      r_gripper_client_(kRightGripperActionName, true),
      arm_joint_client_(kArmJointActionName, true),
      l_arm_joint_client_(kLeftArmJointActionName, true),
      r_arm_joint_client_(kRightArmJointActionName, true) {
  IsValid(action_);
}

bool ActionExecutor::IsValid(const Action& action) {
  if (action.type == Action::ACTUATE_GRIPPER) {
    if (action.actuator_group == Action::GRIPPER ||
        action.actuator_group == Action::LEFT_GRIPPER ||
        action.actuator_group == Action::RIGHT_GRIPPER) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
  } else if (action.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action.actuator_group == Action::ARM ||
        action.actuator_group == Action::LEFT_ARM ||
        action.actuator_group == Action::RIGHT_ARM) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
  } else if (action.type == Action::MOVE_TO_CARTESIAN_GOAL) {
  } else if (action.type == Action::DETECT_TABLETOP_OBJECTS) {
  } else if (action.type == Action::FIND_CUSTOM_LANDMARK) {
  } else {
    ROS_ERROR("Invalid action type: \"%s\"", action.type.c_str());
    return false;
  }
  return true;
}

void ActionExecutor::Start() {
  if (!IsValid(action_)) {
    return;
  }
  if (action_.type == Action::ACTUATE_GRIPPER) {
    ActuateGripper();
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    MoveToJointGoal();
  }
}

bool ActionExecutor::IsDone() const {
  if (!IsValid(action_)) {
    return true;
  }
  if (action_.type == Action::ACTUATE_GRIPPER) {
    if (action_.actuator_group == Action::GRIPPER) {
      return gripper_client_.getState().isDone();
    } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
      return l_gripper_client_.getState().isDone();
    } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
      return r_gripper_client_.getState().isDone();
    }
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action_.actuator_group == Action::ARM) {
      return arm_joint_client_.getState().isDone();
    } else if (action_.actuator_group == Action::LEFT_ARM) {
      return l_arm_joint_client_.getState().isDone();
    } else if (action_.actuator_group == Action::RIGHT_ARM) {
      return r_arm_joint_client_.getState().isDone();
    }
  }
  return true;
}

void ActionExecutor::Cancel() {
  if (!IsValid(action_)) {
    return;
  }
  if (action_.type == Action::ACTUATE_GRIPPER) {
    if (action_.actuator_group == Action::GRIPPER) {
      gripper_client_.cancelAllGoals();
    } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
      l_gripper_client_.cancelAllGoals();
    } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
      r_gripper_client_.cancelAllGoals();
    }
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action_.actuator_group == Action::ARM) {
      arm_joint_client_.cancelAllGoals();
    } else if (action_.actuator_group == Action::LEFT_ARM) {
      l_arm_joint_client_.cancelAllGoals();
    } else if (action_.actuator_group == Action::RIGHT_ARM) {
      r_arm_joint_client_.cancelAllGoals();
    }
  }
}

void ActionExecutor::ActuateGripper() {
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command = action_.gripper_command;
  SimpleActionClient<control_msgs::GripperCommandAction>* client;
  if (action_.actuator_group == Action::GRIPPER) {
    client = &gripper_client_;
  } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
    client = &l_gripper_client_;
  } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
    client = &r_gripper_client_;
  } else {
    return;
  }
  while (!client->waitForServer(ros::Duration(5.0))) {
    ROS_WARN("%s server not running! Trying again in 5 seconds.",
             action_.actuator_group.c_str());
    ros::spinOnce();
  }
  client->sendGoal(gripper_goal);
}

void ActionExecutor::MoveToJointGoal() {
  control_msgs::FollowJointTrajectoryGoal joint_goal;
  joint_goal.trajectory = action_.joint_trajectory;
  joint_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  SimpleActionClient<FollowJointTrajectoryAction>* client;
  if (action_.actuator_group == Action::ARM) {
    client = &arm_joint_client_;
  } else if (action_.actuator_group == Action::LEFT_ARM) {
    client = &l_arm_joint_client_;
  } else if (action_.actuator_group == Action::RIGHT_ARM) {
    client = &r_arm_joint_client_;
  } else {
    return;
  }
  while (!client->waitForServer(ros::Duration(5.0))) {
    ROS_WARN("%s server not running! Trying again in 5 seconds.",
             action_.actuator_group.c_str());
    ros::spinOnce();
  }
  client->sendGoal(joint_goal);
}

void ActionExecutor::PublishInvalidGroupError(const Action& action) {
  ROS_ERROR("Invalid actuator_group \"%s\" for action type \"%s\".",
            action.actuator_group.c_str(), action.type.c_str());
}
}  // namespace pbd
}  // namespace rapid
