#include "rapid_testing/rosout_test_helper.h"

#include "ros/ros.h"

namespace rapid {
RosoutTestHelper::RosoutTestHelper() : nh_(), rosout_sub_(), messages_() {}

void RosoutTestHelper::Start() {
  rosout_sub_ = nh_.subscribe("/rosout", 10, &RosoutTestHelper::Callback, this);
}

void RosoutTestHelper::Clear() { messages_.clear(); }

std::vector<rosgraph_msgs::Log> RosoutTestHelper::messages() const {
  return messages_;
}

bool RosoutTestHelper::WasDebugPublished() const {
  return WasLevelPublished(rosgraph_msgs::Log::DEBUG);
}

bool RosoutTestHelper::WasInfoPublished() const {
  return WasLevelPublished(rosgraph_msgs::Log::INFO);
}

bool RosoutTestHelper::WasWarningPublished() const {
  return WasLevelPublished(rosgraph_msgs::Log::WARN);
}

bool RosoutTestHelper::WasErrorPublished() const {
  return WasLevelPublished(rosgraph_msgs::Log::ERROR);
}

bool RosoutTestHelper::WasFatalPublished() const {
  return WasLevelPublished(rosgraph_msgs::Log::FATAL);
}

void RosoutTestHelper::Callback(const rosgraph_msgs::Log& msg) {
  messages_.push_back(msg);
}

bool RosoutTestHelper::WaitForMessageCount(size_t num_messages,
                                           const ros::Duration& timeout) const {
  ros::Time start = ros::Time::now();
  while (ros::Time::now() < start + timeout) {
    if (num_messages <= messages_.size()) {
      return true;
    }
    ros::spinOnce();
  }
  return false;
}

bool RosoutTestHelper::WasLevelPublished(int8_t level) const {
  for (size_t i = 0; i < messages_.size(); ++i) {
    const rosgraph_msgs::Log& msg = messages_[i];
    if (msg.level == level) {
      return true;
    }
  }
  return false;
}
}  // namespace rapid
