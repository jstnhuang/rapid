#ifndef _RAPID_PBD_MOTION_PLANNING_H_
#define _RAPID_PBD_MOTION_PLANNING_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "rapid_pbd_msgs/Landmark.h"

namespace rapid {
namespace pbd {
class MotionPlanning {
 public:
  MotionPlanning();
  void AddPoseGoal(const std::string& actuator_group,
                   const geometry_msgs::Pose& pose,
                   const rapid_pbd_msgs::Landmark& landmark);
  void AddJointGoal(const std::string& actuator_group,
                    const geometry_msgs::Pose& pose);
  void ClearGoals();

  void Start();
  void IsDone();
  void Cancel();

 private:
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_MOTION_PLANNING_H_
