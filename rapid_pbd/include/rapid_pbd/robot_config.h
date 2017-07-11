#ifndef _RAPID_PBD_ROBOT_CONFIG_H_
#define _RAPID_PBD_ROBOT_CONFIG_H_

#include <string>
#include <vector>

namespace rapid {
namespace pbd {
class RobotConfig {
 public:
  virtual ~RobotConfig() {}
  virtual void planning_frame(std::string* planning_frame) = 0;
  virtual void base_link(std::string* torso_link) = 0;
  virtual void torso_link(std::string* torso_link) = 0;
  virtual void ee_frame_for_group(const std::string& actuator_group,
                                  std::string* ee_frame) = 0;
  virtual void joints_for_group(const std::string& actuator_group,
                                std::vector<std::string>* joint_names) = 0;
  virtual int num_arms() = 0;
  virtual double gripper_open_position() = 0;
};

class Pr2RobotConfig : public RobotConfig {
 public:
  void planning_frame(std::string* planning_frame);
  void base_link(std::string* torso_link);
  void torso_link(std::string* torso_link);
  void ee_frame_for_group(const std::string& actuator_group,
                          std::string* ee_frame);
  void joints_for_group(const std::string& actuator_group,
                        std::vector<std::string>* joint_names);
  int num_arms();
  double gripper_open_position();
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ROBOT_CONFIG_H_
