#ifndef _RAPID_TESTING_ROSOUT_TEST_HELPER_H_
#define _RAPID_TESTING_ROSOUT_TEST_HELPER_H_

#include <vector>

#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"

namespace rapid {
// Helps check that messages were logged in unit tests.
//
// Usage:
//   RosoutTestHelper rosout;
//   rosout.Start();
//   Foo(); // This will publish an error message.
//   rosout.WaitForMessageCount(1, ros::Duration(1.0));
//   EXPECT_TRUE(rosout.WasErrorPublished());
class RosoutTestHelper {
 public:
  RosoutTestHelper();

  // Start subscribing to /rosout.
  void Start();

  // Waits until a certain number of messages have been received.
  // Returns true if the number of received messages within the timeout is at
  // least num_messages, false otherwise.
  bool WaitForMessageCount(size_t num_messages,
                           const ros::Duration& timeout) const;

  // Clear message list.
  void Clear();

  // Get the messages that were published since calling Start();
  std::vector<rosgraph_msgs::Log> messages() const;

  // Returns true if at least one debug-level message was published.
  bool WasDebugPublished() const;

  // Returns true if at least one info-level message was published.
  bool WasInfoPublished() const;

  // Returns true if at least one warning-level message was published.
  bool WasWarningPublished() const;

  // Returns true if at least one error-level message was published.
  bool WasErrorPublished() const;

  // Returns true if at least one fatal-level message was published.
  bool WasFatalPublished() const;

 private:
  void Callback(const rosgraph_msgs::Log& msg);
  bool WasLevelPublished(int8_t level) const;

  ros::NodeHandle nh_;
  ros::Subscriber rosout_sub_;
  std::vector<rosgraph_msgs::Log> messages_;
};
}  // namespace rapid

#endif  // _RAPID_TESTING_ROSOUT_TEST_HELPER_H_
