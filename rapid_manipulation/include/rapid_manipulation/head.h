#ifndef _RAPID_MANIPULATION_HEAD_H_
#define _RAPID_MANIPULATION_HEAD_H_

#include "geometry_msgs/PointStamped.h"
#include "gmock/gmock.h"
#include "pr2_controllers_msgs/PointHeadAction.h"

#include "rapid_ros/action_client.h"

namespace rapid {
namespace manipulation {
class HeadInterface {
 public:
  virtual ~HeadInterface() {}
  virtual bool LookAt(const geometry_msgs::PointStamped& target) = 0;
};

class Head : public HeadInterface {
 public:
  Head(rapid_ros::ActionClientInterface<pr2_controllers_msgs::PointHeadAction>*
           client);
  // Look at the given point. If no frame_id is given, then default to a "head"
  // frame: same orientation as base_footprint but position located near the
  // head.
  bool LookAt(const geometry_msgs::PointStamped& point);

 private:
  rapid_ros::ActionClientInterface<pr2_controllers_msgs::PointHeadAction>*
      client_;
};

class MockHead : public HeadInterface {
 public:
  MOCK_METHOD1(LookAt, bool(const geometry_msgs::PointStamped& target));
};
}  // namespace manipulation
}  // namespace rapid
#endif  // _RAPID_MANIPULATION_HEAD_H_
