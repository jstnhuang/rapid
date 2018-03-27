#include "rapid_ros/time.h"

#include "ros/ros.h"

namespace rapid {
void WaitForTime() {
  while (!ros::Time::isValid()) {
    ros::spinOnce();
  }
}
}  // namespace rapid
