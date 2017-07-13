#include "rapid_pbd/world.h"

#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"

#include "rapid_pbd/joint_state.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
void GetWorld(const msgs::Program& program, size_t step_id, World* world) {
  world->scene_id = "";
  JointState js(program.start_joint_state);
  world->joint_state = js;
  world->surface_box_landmarks.clear();

  for (size_t i = 0; i <= step_id; ++i) {
    if (program.steps.size() == 0) {
      break;
    }
    const msgs::Step& step = program.steps[i];
    if (step.scene_id != "") {
      world->scene_id = step.scene_id;
    }

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
          world->joint_state.SetPosition(name, position);
        }
      } else if (action.type == msgs::Action::MOVE_TO_CARTESIAN_GOAL) {
        // TODO: fill this in.
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
}  // namespace pbd
}  // namespace rapid
