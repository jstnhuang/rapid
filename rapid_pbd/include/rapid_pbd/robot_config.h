#ifndef _RAPID_PBD_ROBOT_CONFIG_H_
#define _RAPID_PBD_ROBOT_CONFIG_H_

#include <string>
#include <vector>

namespace rapid {
namespace pbd {
class RobotConfig {
 public:
  virtual ~RobotConfig() {}
  virtual std::string planning_frame() = 0;
  virtual std::string planning_group() = 0;
  virtual std::string base_link() = 0;
  virtual std::string torso_link() = 0;
  virtual std::string ee_frame_for_group(const std::string& actuator_group) = 0;
  virtual void joints_for_group(const std::string& actuator_group,
                                std::vector<std::string>* joint_names) = 0;
  virtual int num_arms() = 0;
  virtual double gripper_open_position() = 0;
};

class Pr2RobotConfig : public RobotConfig {
 public:
  std::string planning_frame();
  std::string planning_group();
  std::string base_link();
  std::string torso_link();
  std::string ee_frame_for_group(const std::string& actuator_group);
  void joints_for_group(const std::string& actuator_group,
                        std::vector<std::string>* joint_names);
  int num_arms();
  double gripper_open_position();
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ROBOT_CONFIG_H_
