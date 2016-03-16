#ifndef _RAPID_PR2_HEAD_H_
#define _RAPID_PR2_HEAD_H_

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "gmock/gmock.h"
#include "pr2_controllers_msgs/PointHeadAction.h"

namespace rapid {
namespace pr2 {
class HeadInterface {
 public:
  virtual ~HeadInterface() {}
  // Makes the robot look, to the extent of its joint limits, some number of
  // degrees up and to the right.
  //
  // Args:
  //  up: 0 degrees means looking straight ahead, positive means up, negative
  //    means down.
  //  left: 0 degrees means looking straight ahead, positive means left,
  //    negative means right.
  //  virtual bool LookAtDegrees(double up, double left) = 0;
  virtual bool LookAt(const geometry_msgs::PointStamped& target) = 0;
};

// Computes a "look at" point given two degrees. There are no guarantees as to
// how far the look at point will be from the origin.
//
// Args:
//  up: 0 degrees means looking straight ahead, positive means up, negative
//    means down.
//  left: 0 degrees means looking straight ahead, positive means left,
//    negative means right.
// void ComputeLookAtPointFromDegrees(const double up, const double left,
//                                   geometry_msgs::Point* point);

class Head : public HeadInterface {
 public:
  Head();
  //  virtual bool LookAtDegrees(const double up, const double left);
  virtual bool LookAt(const geometry_msgs::PointStamped& point);

 private:
  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>
      head_client_;
};

class MockHead : public HeadInterface {
 public:
  MOCK_METHOD1(LookAt, bool(const geometry_msgs::PointStamped& target));
};
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_HEAD_H_
