#include "rapid_pbd/world.h"

#include <math.h>
#include <set>
#include <string>
#include <vector>

#include "moveit_msgs/GetPositionIK.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/action_utils.h"
#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/robot_config.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
void GetWorld(const RobotConfig& robot_config, const msgs::Program& program,
              size_t step_id, World* world) {
  world->scene_id = "";
  JointState js(program.start_joint_state);
  world->joint_state = js;
  world->surface_box_landmarks.clear();

  // TODO: If this gets noticeably slow, change it so that it searches backward
  // instead of simulating forward. The channels to search are:
  // - Gripper action / actuator group
  // - Joint/Cartesian arm action / actuator group
  // - Scene ID
  // - Landmarks
  for (size_t step_i = 0; step_i <= step_id; ++step_i) {
    if (program.steps.size() == 0) {
      break;
    }
    const msgs::Step& step = program.steps[step_i];
    if (step.scene_id != "") {
      world->scene_id = step.scene_id;
    }

    for (size_t action_i = 0; action_i < step.actions.size(); ++action_i) {
      const msgs::Action& action = step.actions[action_i];
      if (action.type == msgs::Action::ACTUATE_GRIPPER) {
        std::vector<std::string> joint_names;
        robot_config.gripper_joints_for_group(action.actuator_group,
                                              &joint_names);
        std::vector<double> positions;
        if (action.gripper_command.position > 0.0001) {
          robot_config.gripper_open_positions(&positions);
        } else {
          robot_config.gripper_close_positions(&positions);
        }
        for (size_t i = 0; i < joint_names.size(); ++i) {
          world->joint_state.SetPosition(joint_names[i], positions[i]);
        }
      } else if (action.type == msgs::Action::MOVE_TO_JOINT_GOAL) {
        const trajectory_msgs::JointTrajectory& trajectory =
            action.joint_trajectory;
        if (trajectory.points.size() == 0) {
          continue;
        }
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
          const std::string& name = trajectory.joint_names[i];
          double position = trajectory.points[0].positions[i];
          world->joint_state.SetPosition(name, position);
        }
      } else if (action.type == msgs::Action::MOVE_TO_CARTESIAN_GOAL) {
        if (action.landmark.type == "") {
          continue;
        }

        transform_graph::Graph graph;
        graph.Add("landmark",
                  transform_graph::RefFrame(robot_config.base_link()),
                  action.landmark.pose_stamped.pose);
        graph.Add("ee", transform_graph::RefFrame("landmark"), action.pose);
        transform_graph::Transform ee_in_base;
        graph.ComputeDescription(
            transform_graph::LocalFrame("ee"),
            transform_graph::RefFrame(robot_config.base_link()), &ee_in_base);
        geometry_msgs::Pose pose;
        ee_in_base.ToPose(&pose);

        // Get joint angles for pose
        std::vector<std::string> joint_names;
        std::vector<double> joint_positions;

        // Use IK seed if available, compute IK otherwise.
        if (HasJointValues(action)) {
          GetJointPositions(action, &joint_names, &joint_positions);
        } else {
          ROS_WARN(
              "Step %ld, action %ld is old and does not have an IK seed. "
              "Consider remaking it.",
              step_i, action_i);
          moveit_msgs::GetPositionIKRequest ik_req;
          if (action.actuator_group == msgs::Action::ARM) {
            ik_req.ik_request.group_name = "arm";
          } else if (action.actuator_group == msgs::Action::LEFT_ARM) {
            ik_req.ik_request.group_name = "left_arm";
          } else if (action.actuator_group == msgs::Action::RIGHT_ARM) {
            ik_req.ik_request.group_name = "right_arm";
          }
          ik_req.ik_request.pose_stamped.header.frame_id =
              robot_config.base_link();
          ik_req.ik_request.pose_stamped.pose = pose;
          ik_req.ik_request.attempts = 3;
          ik_req.ik_request.timeout = ros::Duration(1);
          robot_config.joints_for_group(action.actuator_group,
                                        &ik_req.ik_request.ik_link_names);

          moveit_msgs::GetPositionIKResponse ik_res;
          ros::service::call("/compute_ik", ik_req, ik_res);
          bool success =
              ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

          if (!success) {
            ROS_ERROR_STREAM("Failed to compute IK for actuator "
                             << action.actuator_group << " on step " << step_i
                             << ", action " << action_i << ", pose: " << pose);
            continue;
          }

          std::set<std::string> joint_set;
          joint_set.insert(ik_req.ik_request.ik_link_names.begin(),
                           ik_req.ik_request.ik_link_names.end());

          for (size_t j = 0; j < ik_res.solution.joint_state.name.size(); ++j) {
            const std::string& name = ik_res.solution.joint_state.name[j];
            if (joint_set.find(name) != joint_set.end()) {
              const double value = ik_res.solution.joint_state.position[j];
              joint_names.push_back(name);
              joint_positions.push_back(value);
            }
          }
        }

        for (size_t i = 0; i < joint_names.size(); ++i) {
          world->joint_state.SetPosition(joint_names[i], joint_positions[i]);
        }
      }
    }

    std::vector<msgs::Landmark> surface_boxes;
    for (size_t landmark_i = 0; landmark_i < step.landmarks.size();
         ++landmark_i) {
      const msgs::Landmark& landmark = step.landmarks[landmark_i];
      if (landmark.type == msgs::Landmark::SURFACE_BOX) {
        surface_boxes.push_back(landmark);
      }
      // Deal with other landmark types here
    }
    // Replace all surface objects if there are any.
    if (surface_boxes.size() > 0) {
      world->surface_box_landmarks = surface_boxes;
    }
  }
}

namespace {
// TODO: move this into a landmark_utils library.
// Returns dimensions as a vector, with x always <= y.
void GetDims(const msgs::Landmark& landmark, std::vector<double>* dims) {
  dims->resize(3);
  dims->at(0) =
      std::min(landmark.surface_box_dims.x, landmark.surface_box_dims.y);
  dims->at(1) =
      std::max(landmark.surface_box_dims.x, landmark.surface_box_dims.y);
  dims->at(2) = landmark.surface_box_dims.z;
}

// Returns squared norm of vectors a and b.
double BoxDissimilarity(const std::vector<double>& a,
                        const std::vector<double>& b) {
  double dx = (a[0] - b[0]);
  double dy = (a[1] - b[1]);
  double dz = (a[2] - b[2]);
  return dx * dx + dy * dy + dz * dz;
}
}

bool MatchLandmark(const World& world, const rapid_pbd_msgs::Landmark& landmark,
                   rapid_pbd_msgs::Landmark* match) {
  const double kMaxDistance = 0.075 * 0.075;
  std::vector<double> landmark_dims;
  GetDims(landmark, &landmark_dims);
  if (landmark.type == msgs::Landmark::SURFACE_BOX) {
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
      const msgs::Landmark& world_landmark = world.surface_box_landmarks[i];
      std::vector<double> world_landmark_dims;
      GetDims(world_landmark, &world_landmark_dims);
      double distance = BoxDissimilarity(landmark_dims, world_landmark_dims);
      if (distance < best) {
        best = distance;
        *match = world_landmark;
      }
    }
    return best <= kMaxDistance;
  } else {
    return false;
  }
}
}  // namespace pbd
}  // namespace rapid
