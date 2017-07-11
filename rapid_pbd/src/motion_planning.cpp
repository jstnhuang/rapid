#include "rapid_pbd/motion_planning.h"

namespace rapid {
namespace pbd {
MotionPlanning::MotionPlanning() {}
void MotionPlanning::AddPoseGoal(const std::string& actuator_group,
                                 const geometry_msgs::Pose& pose,
                                 const rapid_pbd_msgs::Landmark& landmark) {}

void MotionPlanning::AddJointGoal(const std::string& actuator_group,
                                  const geometry_msgs::Pose& pose) {}

void MotionPlanning::ClearGoals() {}

void MotionPlanning::Start() {}

void MotionPlanning::IsDone() {}

void MotionPlanning::Cancel() {}
}  // namespace pbd
}  // namespace rapid
