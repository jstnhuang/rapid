#include "rapid_pbd/editor.h"

#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/SegmentSurfacesGoal.h"
#include "rapid_pbd_msgs/Step.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/visualizer.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
Editor::Editor(const ProgramDb& db, const JointStateReader& joint_state_reader,
               const Visualizer& visualizer, ActionClients* action_clients)
    : db_(db),
      joint_state_reader_(joint_state_reader),
      viz_(visualizer),
      action_clients_(action_clients) {}

void Editor::Start() {
  db_.Start();
  joint_state_reader_.Start();
  viz_.Init();
}

void Editor::HandleEvent(const msgs::EditorEvent& event) {
  if (event.type == msgs::EditorEvent::CREATE) {
    msgs::Program program;
    program.name = event.program_info.name;
    joint_state_reader_.ToMsg(&program.start_joint_state);
    db_.Insert(program);
    viz_.Publish(event.program_info.db_id, 0);
  } else if (event.type == msgs::EditorEvent::UPDATE) {
    Update(event.program_info.db_id, event.program);
  } else if (event.type == msgs::EditorEvent::DELETE) {
    db_.Delete(event.program_info.db_id);
    viz_.StopPublishing(event.program_info.db_id);
  } else if (event.type == msgs::EditorEvent::VIEW) {
    db_.StartPublishingProgramById(event.program_info.db_id);
    viz_.Publish(event.program_info.db_id, event.step_num);
  } else if (event.type == msgs::EditorEvent::DELETE_STEP) {
    DeleteStep(event.program_info.db_id, event.step_num);
  } else if (event.type == msgs::EditorEvent::DETECT_SURFACE_OBJECTS) {
    DetectSurfaceObjects(event.program_info.db_id, event.step_num);
  } else {
    ROS_ERROR("Unknown event type \"%s\"", event.type.c_str());
  }
}

void Editor::Update(const std::string& db_id, const msgs::Program& program) {
  db_.Update(db_id, program);
  viz_.Update(db_id, program);
}

void Editor::DeleteStep(const std::string& db_id, size_t step_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete step from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to delete step %ld from program \"%s\", which has %ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  if (program.steps[step_id].scene_id != "") {
    // TODO: delete the cloud if one exists.
  }
  program.steps.erase(program.steps.begin() + step_id);
  Update(db_id, program);
}

void Editor::DetectSurfaceObjects(const std::string& db_id, size_t step_id) {
  msgs::SegmentSurfacesGoal goal;
  goal.save_cloud = true;
  action_clients_->surface_segmentation_client.sendGoal(goal);
  bool success = action_clients_->surface_segmentation_client.waitForResult(
      ros::Duration(10));
  if (!success) {
    ROS_ERROR("Failed to segment surface.");
    return;
  }
  msgs::SegmentSurfacesResult::ConstPtr result =
      action_clients_->surface_segmentation_client.getResult();

  msgs::Program program;
  success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to update scene for program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to update scene for step %ld, program \"%s\", which has %ld "
        "steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  if (program.steps[step_id].scene_id != "") {
    // TODO: delete the cloud if one exists.
  }
  program.steps[step_id].scene_id = result->cloud_db_id;
  program.steps[step_id].landmarks.insert(
      program.steps[step_id].landmarks.end(), result->landmarks.begin(),
      result->landmarks.end());
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
