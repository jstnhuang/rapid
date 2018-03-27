#ifndef _RAPID_ROS_TIME_H_
#define _RAPID_ROS_TIME_H_

namespace rapid {
/// \brief Wait for simulated time to begin.
///
/// Blocks until the current time is non-zero.
void WaitForTime();
}  // namespace rapid

#endif  // _RAPID_ROS_TIME_H_
