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
  virtual void gripper_joints_for_group(
      const std::string& actuator_group,
      std::vector<std::string>* joint_names) const = 0;
  // In the same order as given by gripper_joints_for_group
  virtual void gripper_open_positions(std::vector<double>* positions) const = 0;
  virtual void gripper_close_positions(
      std::vector<double>* positions) const = 0;
  virtual void joints_for_group(
      const std::string& actuator_group,
      std::vector<std::string>* joint_names) const = 0;
  virtual int num_arms() const = 0;
};

class Pr2RobotConfig : public RobotConfig {
 public:
  Pr2RobotConfig();
  std::string planning_frame() const;
  std::string planning_group() const;
  std::string base_link() const;
  std::string torso_link() const;
  std::string ee_frame_for_group(const std::string& actuator_group) const;
  void gripper_joints_for_group(const std::string& actuator_group,
                                std::vector<std::string>* joint_names) const;
  void gripper_open_positions(std::vector<double>* positions) const;
  void gripper_close_positions(std::vector<double>* positions) const;
  void joints_for_group(const std::string& actuator_group,
                        std::vector<std::string>* joint_names) const;
  int num_arms() const;
};

class FetchRobotConfig : public RobotConfig {
 public:
  FetchRobotConfig();
  std::string planning_frame() const;
  std::string planning_group() const;
  std::string base_link() const;
  std::string torso_link() const;
  std::string ee_frame_for_group(const std::string& actuator_group) const;
  void gripper_joints_for_group(const std::string& actuator_group,
                                std::vector<std::string>* joint_names) const;
  void gripper_open_positions(std::vector<double>* positions) const;
  void gripper_close_positions(std::vector<double>* positions) const;
  void joints_for_group(const std::string& actuator_group,
                        std::vector<std::string>* joint_names) const;
  int num_arms() const;
};

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ROBOT_CONFIG_H_
