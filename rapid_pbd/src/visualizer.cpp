#include "rapid_pbd/visualizer.h"

#include <map>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "robot_state_publisher_latched/robot_state_publisher_latched.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
Visualizer::Visualizer(
    const robot_state_publisher_latched::RobotStatePublisher& robot_state_pub)
    : robot_state_pub_(robot_state_pub) {}

// Updates the state of the visualization.
void Visualizer::HandleUpdate(const msgs::EditorEvent& event) {
  // Update the joint state for each step.
  const std::string& db_id = event.program_info.db_id;

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
    const std::string& tf_prefix = TfPrefix(db_id, step_i);
    std::map<std::string, double> joint_positions;
    current.ToMap(&joint_positions);

    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    robot_state_pub_.getTransforms(joint_positions, ros::Time::now(), tf_prefix,
                                   &tf_transforms);
    geometry_msgs::TransformStamped identity;
    identity.header.frame_id = "base_link";
    identity.header.stamp = ros::Time::now();
    identity.child_frame_id = tf_prefix + "/base_link";
    identity.transform.rotation.w = 1;
    tf_transforms.push_back(identity);
    robot_state_pub_.publishTransforms(tf_transforms);
  }
}

std::string Visualizer::TfPrefix(const std::string& db_id, const int step_id) {
  std::stringstream ss;
  ss << "rapid_pbd/" << db_id << "/" << step_id;
  return ss.str();
}

}  // namespace pbd
}  // namespace rapid
