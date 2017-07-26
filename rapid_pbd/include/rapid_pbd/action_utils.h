#ifndef _RAPID_PBD_ACTION_UTILS_H_
#define _RAPID_PBD_ACTION_UTILS_H_

#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"

namespace rapid {
namespace pbd {
// Gets the first point of the joint trajectory specified in the action.
bool HasJointValues(const rapid_pbd_msgs::Action& action);

// Gets the joint positions for the first point of the action's joint
// trajectory.
void GetJointPositions(const rapid_pbd_msgs::Action& action,
                       std::vector<std::string>* joint_names,
                       std::vector<double>* joint_positions);

// Sets the joint positions for the first point of the action's joint
// trajectory.
void SetJointPositions(const std::vector<std::string>& joint_names,
                       const std::vector<double>& joint_positions,
                       rapid_pbd_msgs::Action* action);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_UTILS_H_
