#ifndef _RAPID_ROBOT_JOINT_STATE_READER_H_
#define _RAPID_ROBOT_JOINT_STATE_READER_H_

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace rapid {
/// \brief Tracks the latest joint states.
///
/// Currently, we only track joint positions/angles, not velocities or efforts.
///
/// Usage:
/// \code
/// JointStateReader js_reader;
/// js_reader.Start();
/// ...
/// if (js_reader.HasJoint("torso_lift_link")) {
///   double torso_pos = js_reader.position("torso_lift_link");
/// }
/// std::map joint_positions = js_reader.positions();
/// \endcode
class JointStateReader {
 public:
  /// \brief Constructor
  JointStateReader();

  /// \brief Constructor with a different joint states topic.
  JointStateReader(const std::string& joint_states_topic);

  /// \brief Starts subscribing to the joint states.
  ///
  /// The JointStateReader will not work until you call Start().
  void Start();

  /// \brief Returns true if this reader has a given joint state.
  ///
  /// \param[in] name The name of the joint to check.
  ///
  /// \returns true if the reader has received at least one value from the
  ///   joint_states topic for the given joint, false otherwise.
  bool HasJoint(const std::string& name) const;

  /// \brief Returns the position of the given joint.
  ///
  /// \param[in] name The name of the joint.
  ///
  /// \throws std::out_of_range if the joint does not exist.
  double position(const std::string& name) const;

  /// \brief Returns all the joint positions so far.
  std::map<std::string, double> positions() const;

 private:
  void callback(const sensor_msgs::JointState& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string topic_;
  std::map<std::string, double> positions_;
};
}  // namespace rapid

#endif  // _RAPID_ROBOT_JOINT_STATE_READER_H_
