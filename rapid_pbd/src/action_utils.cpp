#include "rapid_pbd/action_utils.h"

#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"

namespace rapid {
namespace pbd {
bool HasJointValues(const rapid_pbd_msgs::Action& action) {
  return action.joint_trajectory.points.size() > 0;
}

void GetJointPositions(const rapid_pbd_msgs::Action& action,
                       std::vector<std::string>* joint_names,
                       std::vector<double>* joint_positions) {
  if (!HasJointValues(action)) {
    return;
  }

  *joint_names = action.joint_trajectory.joint_names;
  const trajectory_msgs::JointTrajectoryPoint& pt =
      action.joint_trajectory.points[0];
  *joint_positions = pt.positions;
}

void SetJointPositions(const std::vector<std::string>& joint_names,
                       const std::vector<double>& joint_positions,
                       rapid_pbd_msgs::Action* action) {
  action->joint_trajectory.joint_names = joint_names;

  if (action->joint_trajectory.points.size() == 0) {
    action->joint_trajectory.points.resize(1);
  }
  action->joint_trajectory.points[0].positions = joint_positions;
}
}  // namespace pbd
}  // namespace rapid

int main(int argc, char** argv) { return 0; }
