// A class for opening and closing the grippers.
//
// Sample usage:
//  rapid::manipulation::Gripper right_gripper(
//    Gripper::RIGHT_GRIPPER);
//  right_gripper.Open();
//  right_gripper.Close();
//
// Notes:
// 1. The gripper does not work quite right in Gazebo
// 2. Return value is based on whether the gripper reached the target position.
//    So, if the gripper is closing around an object, Close() will typically
//    return false because the object stalls the gripper before it reaches
//    the goal position.

#ifndef _RAPID_MANIPULATION_GRIPPER_H_
#define _RAPID_MANIPULATION_GRIPPER_H_

#include "actionlib/client/simple_action_client.h"
#include "gmock/gmock.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_perception/object.h"
#include "rapid_ros/action_client.h"

namespace rapid {
namespace manipulation {
typedef rapid_ros::ActionClientInterface<
    pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class GripperInterface {
 public:
  virtual ~GripperInterface() {}

  // Gets the gripper to the given position.
  // position - how wide to open or close the gripper
  // effort - now much force to exert, negative is full force
  virtual bool SetPosition(double position, double effort = -1) = 0;

  // Gets the gripper's current position. Note: may not agree with "SetPosition"
  virtual double GetPosition() const = 0;

  // Returns true if the gripper is holding an object, false otherwise.
  virtual bool HeldObject(rapid::perception::Object* object) const = 0;

  // Returns whether the gripper is open or not.
  virtual bool IsOpen() const = 0;

  // Opens the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  // effort - defaults to -1.0, to open with unlimited effort
  virtual bool Open(double effort = -1.0) = 0;

  // Closes the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  // effort - defaults to 50.0 to close gently
  virtual bool Close(double effort = -1.0) = 0;

  // Sets the held object.
  virtual void set_held_object(const rapid::perception::Object& object) = 0;

  // Returns true if the gripper is holding an object, false otherwise.
  virtual bool is_holding_object() const = 0;

  virtual void set_is_holding_object(bool holding) = 0;
};

// Implementation of the GripperInterface for the PR2.
class Gripper : public GripperInterface {
 public:
  static const double OPEN;    // Canonical "open" position.
  static const double CLOSED;  // Canonical "closed" position.

  // Gripper ids
  static const int LEFT_GRIPPER = 0;
  static const int RIGHT_GRIPPER = 1;

  // Gripper action topics
  static const std::string LEFT_GRIPPER_ACTION;
  static const std::string RIGHT_GRIPPER_ACTION;

  // Constructor that takes the gripper id and the gripper client.
  // gripper_id: Gripper::LEFT_GRIPPER or Gripper::RIGHT_GRIPPER
  // client: a rapid_ros::ActionClient
  Gripper(const int gripper_id, GripperClient* client);

  ~Gripper();

  bool SetPosition(double position, double effort = -1.0);
  double GetPosition() const;
  bool HeldObject(rapid::perception::Object* object) const;
  bool IsOpen() const;
  bool Open(double effort = -1.0);
  bool Close(double effort = -1.0);

  bool is_holding_object() const;
  void set_held_object(const rapid::perception::Object& object);
  void set_is_holding_object(bool holding);

 private:
  const int gripper_id_;
  GripperClient* gripper_client_;
  tf::TransformListener transform_listener_;
  bool is_holding_object_;
  rapid::perception::Object held_object_;

  static const double OPEN_THRESHOLD;  // Gripper open threshold
};

class MockGripper : public GripperInterface {
 public:
  MOCK_METHOD2(SetPosition, bool(double position, double effort));
  MOCK_CONST_METHOD0(GetPosition, double());
  MOCK_CONST_METHOD1(HeldObject, bool(rapid::perception::Object*));
  MOCK_CONST_METHOD0(IsOpen, bool());
  MOCK_METHOD1(Open, bool(double effort));
  MOCK_METHOD1(Close, bool(double effort));
  MOCK_CONST_METHOD0(is_holding_object, bool());
  MOCK_METHOD1(set_is_holding_object, void(bool));
  MOCK_METHOD1(set_held_object, void(const rapid::perception::Object&));
};
}  // namespace manipulation
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_GRIPPER_H_
