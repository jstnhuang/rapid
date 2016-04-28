#include "rapid_ros/tf_listener.h"

#include <iostream>
#include <map>
#include <string>
#include <utility>

#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "ros/duration.h"
#include "ros/time.h"

namespace rapid_ros {
TfListener::TfListener() : tf_() {}

bool TfListener::waitForTransform(const std::string& target_frame,
                                  const std::string& source_frame,
                                  const ros::Time& time,
                                  const ros::Duration& timeout) const {
  return tf_.waitForTransform(target_frame, source_frame, time, timeout);
}

void TfListener::lookupTransform(const std::string& target_frame,
                                 const std::string& source_frame,
                                 const ros::Time& time,
                                 tf::StampedTransform& transform) const {
  tf_.lookupTransform(target_frame, source_frame, time, transform);
}

MockTfListener::MockTfListener()
    : transforms_(), durations_(), should_throw_exception_() {}

bool MockTfListener::waitForTransform(const std::string& target_frame,
                                      const std::string& source_frame,
                                      const ros::Time& time,
                                      const ros::Duration& timeout) const {
  FramePair fp(target_frame, source_frame);
  if (should_throw_exception_.find(fp) != should_throw_exception_.end() &&
      should_throw_exception_.find(fp)->second == false) {
    throw tf::TransformException("Mock exception");
  }

  if (timeout.isZero()) {
    std::cerr << "Timeout was set to infinite! Failing waitForTransform"
              << std::endl;
    return false;  // It's always wrong to wait for infinite time.
  }

  if (durations_.find(fp) != durations_.end() &&
      durations_.find(fp)->second >= timeout) {
    return false;  // Timeout exceeded.
  }

  if (transforms_.find(fp) == transforms_.end()) {
    throw tf::LookupException("Missing transform");
  } else {
    return true;
  }
}

void MockTfListener::lookupTransform(const std::string& target_frame,
                                     const std::string& source_frame,
                                     const ros::Time& time,
                                     tf::StampedTransform& transform) const {
  FramePair fp(target_frame, source_frame);
  if (should_throw_exception_.find(fp) != should_throw_exception_.end() &&
      should_throw_exception_.find(fp)->second == false) {
    throw tf::TransformException("Mock exception");
  }

  if (transforms_.find(fp) == transforms_.end()) {
    throw tf::LookupException("Missing transform");
  } else {
    transform = transforms_.find(fp)->second;
  }
}

void MockTfListener::SetTransform(const std::string& target,
                                  const std::string& source,
                                  const tf::StampedTransform& transform) {
  FramePair fp(target, source);
  transforms_[fp] = transform;
}

void MockTfListener::SetLookupDuration(const std::string& target,
                                       const std::string& source,
                                       const ros::Duration& duration) {
  FramePair fp(target, source);
  durations_[fp] = duration;
}

void MockTfListener::SetThrowException(const std::string& target,
                                       const std::string& source,
                                       bool should_throw) {
  FramePair fp(target, source);
  should_throw_exception_[fp] = should_throw;
}
}  // namespace rapid_ros
