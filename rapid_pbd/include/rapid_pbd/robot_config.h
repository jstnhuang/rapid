#ifndef _RAPID_PBD_ROBOT_CONFIG_H_
#define _RAPID_PBD_ROBOT_CONFIG_H_

#include <string>
#include <vector>

namespace rapid {
namespace pbd {
class RobotConfig {
 public:
  virtual ~RobotConfig() {}
  virtual std::string planning_frame() const = 0;
  virtual std::string planning_group() const = 0;
  virtual std::string base_link() const = 0;
  virtual std::string torso_link() const = 0;
  virtual std::string ee_frame_for_group(
      const std::string& actuator_group) const = 0;
  virtual void joints_for_group(
      const std::string& actuator_group,
      std::vector<std::string>* joint_names) const = 0;
  virtual int num_arms() const = 0;
  virtual double gripper_open_position() const = 0;
};

class Pr2RobotConfig : public RobotConfig {
 public:
  std::string planning_frame() const;
  std::string planning_group() const;
  std::string base_link() const;
  std::string torso_link() const;
  std::string ee_frame_for_group(const std::string& actuator_group) const;
  void joints_for_group(const std::string& actuator_group,
                        std::vector<std::string>* joint_names) const;
  int num_arms() const;
  double gripper_open_position() const;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ROBOT_CONFIG_H_
