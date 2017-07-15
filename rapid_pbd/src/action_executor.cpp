#include "rapid_pbd/action_executor.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_pbd/action_names.h"
#include "rapid_pbd/errors.h"
#include "rapid_pbd/motion_planning.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using control_msgs::FollowJointTrajectoryAction;
using rapid_pbd_msgs::Action;

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
ActionExecutor::ActionExecutor(const Action& action,
                               ActionClients* action_clients,
                               MotionPlanning* motion_planning, World* world,
                               const RobotConfig& robot_config,
                               const RuntimeVisualizer& runtime_viz)
    : action_(action),
      clients_(action_clients),
      motion_planning_(motion_planning),
      world_(world),
      robot_config_(robot_config),
      runtime_viz_(runtime_viz) {}

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
        action.actuator_group == Action::RIGHT_ARM ||
        action.actuator_group == Action::HEAD) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
  } else if (action.type == Action::MOVE_TO_CARTESIAN_GOAL) {
    if (action.actuator_group == Action::ARM ||
        action.actuator_group == Action::LEFT_ARM ||
        action.actuator_group == Action::RIGHT_ARM ||
        action.actuator_group == Action::HEAD) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
  } else if (action.type == Action::DETECT_TABLETOP_OBJECTS) {
  } else if (action.type == Action::FIND_CUSTOM_LANDMARK) {
  } else {
    ROS_ERROR("Invalid action type: \"%s\"", action.type.c_str());
    return false;
  }
  return true;
}

std::string ActionExecutor::Start() {
  if (action_.type == Action::ACTUATE_GRIPPER) {
    ActuateGripper();
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    MoveToJointGoal();
  } else if (action_.type == Action::MOVE_TO_CARTESIAN_GOAL) {
    return motion_planning_->AddPoseGoal(action_.actuator_group, action_.pose,
                                         action_.landmark);
  } else if (action_.type == Action::DETECT_TABLETOP_OBJECTS) {
    DetectTabletopObjects();
  }
  return "";
}

bool ActionExecutor::IsDone(std::string* error) const {
  if (action_.type == Action::ACTUATE_GRIPPER) {
    if (action_.actuator_group == Action::GRIPPER) {
      return clients_->gripper_client.getState().isDone();
    } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
      return clients_->l_gripper_client.getState().isDone();
    } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
      return clients_->r_gripper_client.getState().isDone();
    }
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action_.actuator_group == Action::ARM) {
      return clients_->arm_joint_client.getState().isDone();
    } else if (action_.actuator_group == Action::LEFT_ARM) {
      return clients_->l_arm_joint_client.getState().isDone();
    } else if (action_.actuator_group == Action::RIGHT_ARM) {
      return clients_->r_arm_joint_client.getState().isDone();
    } else if (action_.actuator_group == Action::HEAD) {
      return clients_->head_client.getState().isDone();
    }
  } else if (action_.type == Action::DETECT_TABLETOP_OBJECTS) {
    bool done = clients_->surface_segmentation_client.getState().isDone();
    if (done) {
      msgs::SegmentSurfacesResultConstPtr result =
          clients_->surface_segmentation_client.getResult();
      if (result) {
        if (result->landmarks.size() == 0) {
          *error = errors::kNoLandmarksDetected;
        }
        world_->surface_box_landmarks = result->landmarks;
        runtime_viz_.PublishSurfaceBoxes(result->landmarks);
      } else {
        ROS_ERROR("Surface segmentation result pointer was null!");
        *error = "Surface segmentation result pointer was null!";
        return false;
      }
    }
    return done;
  }
  return true;
}

void ActionExecutor::Cancel() {
  if (action_.type == Action::ACTUATE_GRIPPER) {
    if (action_.actuator_group == Action::GRIPPER) {
      clients_->gripper_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
      clients_->l_gripper_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
      clients_->r_gripper_client.cancelAllGoals();
    }
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action_.actuator_group == Action::ARM) {
      clients_->arm_joint_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::LEFT_ARM) {
      clients_->l_arm_joint_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::RIGHT_ARM) {
      clients_->r_arm_joint_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::HEAD) {
      clients_->head_client.cancelAllGoals();
    }
  } else if (action_.type == Action::DETECT_TABLETOP_OBJECTS) {
    clients_->surface_segmentation_client.cancelAllGoals();
  }
}

void ActionExecutor::ActuateGripper() {
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command = action_.gripper_command;

  SimpleActionClient<control_msgs::GripperCommandAction>* client;
  if (action_.actuator_group == Action::GRIPPER) {
    client = &clients_->gripper_client;
  } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
    client = &clients_->l_gripper_client;
  } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
    client = &clients_->r_gripper_client;
  } else {
    return;
  }
  client->sendGoal(gripper_goal);
}

void ActionExecutor::MoveToJointGoal() {
  control_msgs::FollowJointTrajectoryGoal joint_goal;
  joint_goal.trajectory = action_.joint_trajectory;
  joint_goal.trajectory.header.stamp = ros::Time::now();

  // Delay the action if moving the arm
  if (action_.actuator_group == Action::ARM ||
      action_.actuator_group == Action::LEFT_ARM ||
      action_.actuator_group == Action::RIGHT_ARM) {
    double delay;
    ros::param::param("arm_delay", delay, 1.5);
    joint_goal.trajectory.header.stamp =
        ros::Time::now() + ros::Duration(delay);
  }

  SimpleActionClient<FollowJointTrajectoryAction>* client;
  if (action_.actuator_group == Action::ARM) {
    client = &clients_->arm_joint_client;
  } else if (action_.actuator_group == Action::LEFT_ARM) {
    client = &clients_->l_arm_joint_client;
  } else if (action_.actuator_group == Action::RIGHT_ARM) {
    client = &clients_->r_arm_joint_client;
  } else if (action_.actuator_group == Action::HEAD) {
    client = &clients_->head_client;
  } else {
    return;
  }
  client->sendGoal(joint_goal);
}

void ActionExecutor::DetectTabletopObjects() {
  rapid_pbd_msgs::SegmentSurfacesGoal goal;
  goal.save_cloud = false;
  clients_->surface_segmentation_client.sendGoal(goal);
}

void ActionExecutor::PublishInvalidGroupError(const Action& action) {
  ROS_ERROR("Invalid actuator_group \"%s\" for action type \"%s\".",
            action.actuator_group.c_str(), action.type.c_str());
}
}  // namespace pbd
}  // namespace rapid
