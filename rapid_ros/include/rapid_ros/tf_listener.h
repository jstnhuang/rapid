#ifndef _RAPID_ROS_TF_LISTENER_H_
#define _RAPID_ROS_TF_LISTENER_H_

#include <map>
#include <string>
#include <utility>

#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "ros/duration.h"
#include "ros/time.h"

namespace rapid_ros {
// Interface wrapper for a tf::TransformListener.
// Supports only a subset of TransformListener for now, more will be added as
// needed.
class TfListenerInterface {
 public:
  virtual ~TfListenerInterface() {}
  // Waits for a transformation to be available
  virtual bool waitForTransform(const std::string& target_frame,
                                const std::string& source_frame,
                                const ros::Time& time,
                                const ros::Duration& timeout) const = 0;
  virtual void lookupTransform(const std::string& target_frame,
                               const std::string& source_frame,
                               const ros::Time& time,
                               tf::StampedTransform& transform) const = 0;
};

// A wrapper around a real tf::TransformListener.
class TfListener : public TfListenerInterface {
 public:
  TfListener();
  bool waitForTransform(const std::string& target_frame,
                        const std::string& source_frame, const ros::Time& time,
                        const ros::Duration& timeout) const;
  void lookupTransform(const std::string& target_frame,
                       const std::string& source_frame, const ros::Time& time,
                       tf::StampedTransform& transform) const;

 private:
  tf::TransformListener tf_;
};

// A mock tf listener.
//
// For each FramePair (target, source), you can specify the transform to be
// returned, how long it should take to return the transform, and whether or not
// looking up the transform should throw an exception.
//
// If no duration is set for a FramePair, then it is assumed to be 0.
// If the exception flag is not set for a FramePair, then lookups will not throw
// an exception.
class MockTfListener : public TfListenerInterface {
 public:
  typedef std::pair<std::string, std::string> FramePair;

  MockTfListener();

  // Note we do not simulate anything related to the frame's time.
  bool waitForTransform(const std::string& target_frame,
                        const std::string& source_frame, const ros::Time& time,
                        const ros::Duration& timeout) const;

  // Note we do not simulate anything related to the frame's time.
  void lookupTransform(const std::string& target_frame,
                       const std::string& source_frame, const ros::Time& time,
                       tf::StampedTransform& transform) const;

  // Mock methods
  // Sets the transform that should be returned when doing a lookup from the
  // source to the target frame.
  void SetTransform(const std::string& target, const std::string& source,
                    const tf::StampedTransform& transform);

  // Sets how long it should take to perform a lookup from the source to the
  // target frame.
  void SetLookupDuration(const std::string& target, const std::string& source,
                         const ros::Duration& duration);

  // Sets whether or not an exception should be thrown when doing a lookup from
  // the source to the target frame.
  void SetThrowException(const std::string& target, const std::string& source,
                         bool should_throw);

 private:
  std::map<FramePair, tf::StampedTransform> transforms_;
  std::map<FramePair, ros::Duration> durations_;
  std::map<FramePair, bool> should_throw_exception_;
};
}  // namespace rapid_ros

#endif  // _RAPID_ROS_TF_LISTENER_H_
