#ifndef _RAPID_PBD_ACTION_CLIENTS_H_
#define _RAPID_PBD_ACTION_CLIENTS_H_

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"

namespace rapid {
namespace pbd {
struct ActionClients {
 public:
  ActionClients();
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripper_client;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      l_gripper_client;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      r_gripper_client;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      arm_joint_client;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      l_arm_joint_client;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      r_arm_joint_client;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      head_client;
  actionlib::SimpleActionClient<rapid_pbd_msgs::SegmentSurfacesAction>
      surface_segmentation_client;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_CLIENTS_H_
