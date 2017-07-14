#include "rapid_pbd/world.h"

#include <math.h>
#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/joint_state.h"
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
        std::vector<std::string> joint_names;
        std::vector<double> joint_values;

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
        bool success = robot_config.ComputeIk(action.actuator_group, pose,
                                              &joint_names, &joint_values);
        if (!success) {
          ROS_ERROR_STREAM("Failed to compute IK for actuator "
                           << action.actuator_group << "on step" << step_i
                           << ", action " << action_i << ", pose: " << pose);
          continue;
        }
        for (size_t j = 0; j < joint_names.size(); ++j) {
          const std::string& name = joint_names[j];
          double position = joint_values[j];
          world->joint_state.SetPosition(name, position);
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
