#include "rapid_pbd/editor.h"

#include <string>
#include <vector>

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
Editor::Editor(const ProgramDb& db, const JointStateReader& joint_state_reader)
    : db_(db), joint_state_reader_(joint_state_reader) {}

void Editor::Start() {
  db_.Start();
  joint_state_reader_.Start();
}

void Editor::HandleEvent(const msgs::EditorEvent& event) {
  if (event.type == msgs::EditorEvent::CREATE) {
    msgs::Program program;
    program.name = event.program_info.name;
    joint_state_reader_.ToMsg(&program.start_joint_state);
    db_.Insert(program);
  } else if (event.type == msgs::EditorEvent::UPDATE) {
    HandleUpdate(event);
  } else if (event.type == msgs::EditorEvent::DELETE) {
    db_.Delete(event.program_info.db_id);
  } else if (event.type == msgs::EditorEvent::OPEN) {
    db_.StartPublishingProgramById(event.program_info.db_id);
  } else {
    ROS_ERROR("Unknown event type \"%s\"", event.type.c_str());
  }
}

bool Editor::HandleGetEEPose(rapid_pbd_msgs::GetEEPoseRequest& request,
                             rapid_pbd_msgs::GetEEPoseResponse& response) {
  ROS_ERROR("Not implemented");
  return false;
}

bool Editor::HandleGetJointAngles(
    rapid_pbd_msgs::GetJointAnglesRequest& request,
    rapid_pbd_msgs::GetJointAnglesResponse& response) {
  if (request.actuator_group == msgs::Action::ARM) {
    ArmJointNames(&response.joint_names);
  } else if (request.actuator_group == msgs::Action::LEFT_ARM) {
    LeftArmJointNames(&response.joint_names);
  } else if (request.actuator_group == msgs::Action::RIGHT_ARM) {
    RightArmJointNames(&response.joint_names);
  } else {
    ROS_ERROR("Can't get joint angles for actuator group \"%s\"",
              request.actuator_group.c_str());
    return false;
  }
  for (size_t i = 0; i < response.joint_names.size(); ++i) {
    const std::string& name = response.joint_names[i];
    double pos = joint_state_reader_.get_position(name);
    if (pos == kNoJointValue) {
      ROS_ERROR("Could not get angle for joint \"%s\"", name.c_str());
      return false;
    } else {
      response.joint_positions.push_back(pos);
    }
  }

  return true;
}

void Editor::HandleUpdate(const rapid_pbd_msgs::EditorEvent& event) {
  db_.Update(event.program_info.db_id, event.program);
}

void ArmJointNames(std::vector<std::string>* names) {
  names->push_back("shoulder_pan_joint");
  names->push_back("shoulder_lift_joint");
  names->push_back("upperarm_roll_joint");
  names->push_back("elbow_flex_joint");
  names->push_back("forearm_roll_joint");
  names->push_back("wrist_flex_joint");
  names->push_back("wrist_roll_joint");
}

void LeftArmJointNames(std::vector<std::string>* names) {
  names->push_back("l_shoulder_pan_joint");
  names->push_back("l_shoulder_lift_joint");
  names->push_back("l_upper_arm_roll_joint");
  names->push_back("l_elbow_flex_joint");
  names->push_back("l_forearm_roll_joint");
  names->push_back("l_wrist_flex_joint");
  names->push_back("l_wrist_roll_joint");
}

void RightArmJointNames(std::vector<std::string>* names) {
  names->push_back("r_shoulder_pan_joint");
  names->push_back("r_shoulder_lift_joint");
  names->push_back("r_upper_arm_roll_joint");
  names->push_back("r_elbow_flex_joint");
  names->push_back("r_forearm_roll_joint");
  names->push_back("r_wrist_flex_joint");
  names->push_back("r_wrist_roll_joint");
}

}  // namespace pbd
}  // namespace rapid
