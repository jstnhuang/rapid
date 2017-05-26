#include "rapid_pbd/visualizer.h"

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "robot_markers/builder.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"

namespace msgs = rapid_pbd_msgs;
using visualization_msgs::MarkerArray;

namespace rapid {
namespace pbd {
Visualizer::Visualizer(const ProgramDb& db,
                       const robot_markers::Builder& marker_builder)
    : db_(db), marker_builder_(marker_builder), step_vizs_(), nh_() {}

void Visualizer::Init() {
  marker_builder_.Init();
  marker_builder_.SetNamespace("robot");
  marker_builder_.SetFrameId("base_link");
}

void Visualizer::Publish(const std::string& program_id, int step_num) {
  msgs::Program program;
  if (!db_.Get(program_id, &program)) {
    return;
  }
  size_t step_id = static_cast<size_t>(step_num);
  if (step_id > program.steps.size()) {
    ROS_ERROR(
        "Cannot visualize step %ld of program %s, which has only %ld steps.",
        step_id, program_id.c_str(), program.steps.size());
    return;
  }

  // If user just added a step, then visualize the robot in its previous state.
  if (step_id == program.steps.size() && program.steps.size() > 0) {
    step_id = program.steps.size() - 1;
  }

  // Create the publisher if it doesn't exist.
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    step_vizs_[program_id].robot_pub =
        nh_.advertise<MarkerArray>("robot/" + program_id, 10, true);
  }

  // Publish the robot visualization
  MarkerArray robot_markers;
  GetRobotMarker(program, step_id, &robot_markers);
  step_vizs_[program_id].robot_pub.publish(robot_markers);
  step_vizs_[program_id].step_id = step_id;
}

void Visualizer::Update(const std::string& program_id,
                        const rapid_pbd_msgs::Program& program) {
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    return;
  }
  MarkerArray robot_markers;
  GetRobotMarker(program, step_vizs_[program_id].step_id, &robot_markers);
  step_vizs_[program_id].robot_pub.publish(robot_markers);
}

void Visualizer::StopPublishing(const std::string& program_id) {
  if (step_vizs_.find(program_id) != step_vizs_.end()) {
    step_vizs_.erase(program_id);
  }
}

bool Visualizer::GetRobotMarker(const msgs::Program& program, size_t step_id,
                                MarkerArray* robot_markers) {
  // Update the joint state for each step.
  JointState current(program.start_joint_state);
  for (size_t step_i = 0; step_i <= step_id; ++step_i) {
    const msgs::Step& step = program.steps[step_i];
    for (size_t action_i = 0; action_i < step.actions.size(); ++action_i) {
      const msgs::Action& action = step.actions[action_i];
      if (action.type == msgs::Action::ACTUATE_GRIPPER) {
        // TODO: fill this in.
      } else if (action.type == msgs::Action::MOVE_TO_JOINT_GOAL) {
        const trajectory_msgs::JointTrajectory& trajectory =
            action.joint_trajectory;
        if (trajectory.points.size() == 0) {
          continue;
        }
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
          const std::string& name = trajectory.joint_names[i];
          double position = trajectory.points[0].positions[i];
          current.SetPosition(name, position);
        }
      } else if (action.type == msgs::Action::MOVE_TO_CARTESIAN_GOAL) {
        // TODO: fill this in.
      }
    }
    std::map<std::string, double> joint_positions;
    current.ToMap(&joint_positions);

    marker_builder_.SetJointPositions(joint_positions);
    marker_builder_.Build(robot_markers);
  }
  return true;
}
}  // namespace pbd
}  // namespace rapid
