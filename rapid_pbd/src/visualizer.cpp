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

namespace rapid {
namespace pbd {

Visualizer::Visualizer(const robot_markers::Builder& marker_builder)
    : marker_builder_(marker_builder), step_vizs_(), nh_() {}

void Visualizer::Init() { marker_builder_.Init(); }

// Updates the state of the visualization.
void Visualizer::HandleUpdate(const msgs::EditorEvent& event) {
  // Update the joint state for each step.
  JointState current(event.program.start_joint_state);
  for (size_t step_i = 0; step_i < event.program.steps.size(); ++step_i) {
    const msgs::Step& step = event.program.steps[step_i];
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

    ProgramStep step_key(event.program_info.db_id, step_i);
    StepVisualization& step_viz = step_vizs_[step_key];

    marker_builder_.SetNamespace("robot");
    marker_builder_.SetFrameId("base_link");
    marker_builder_.SetJointPositions(joint_positions);
    marker_builder_.Build(&step_viz.robot_arr);
  }
}

void Visualizer::Publish(const std::string& program_id, int step_num) {
  ProgramStep step(program_id, step_num);
  if (program_pubs_.find(program_id) == program_pubs_.end()) {
    program_pubs_[program_id] = nh_.advertise<visualization_msgs::MarkerArray>(
        "robot/" + program_id, 10, true);
  }
  if (step_vizs_.find(step) != step_vizs_.end()) {
    program_pubs_[program_id].publish(step_vizs_[step].robot_arr);
  } else {
    ROS_ERROR("Could not publish visualization for program %s, step %d",
              program_id.c_str(), step_num);
  }
}
}  // namespace pbd
}  // namespace rapid
