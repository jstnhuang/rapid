#ifndef _RAPID_PBD_WORLD_H_
#define _RAPID_PBD_WORLD_H_

#include <string>
#include <vector>

#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
struct World {
 public:
  std::string scene_id;
  JointState joint_state;
  std::vector<rapid_pbd_msgs::Landmark> surface_box_landmarks;
};

void GetWorld(const RobotConfig& robot_config,
              const rapid_pbd_msgs::Program& program, size_t step_id,
              World* world);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
