#ifndef _RAPID_PBD_EDITOR_H_
#define _RAPID_PBD_EDITOR_H_

#include <string>
#include <vector>

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/GetEEPose.h"
#include "rapid_pbd_msgs/GetJointAngles.h"

namespace rapid {
namespace pbd {

static const char kEditorEventsTopic[] = "editor_events";

// Assumed names of joints for a single-arm robot (based on Fetch).
void ArmJointNames(std::vector<std::string>* names);

// Assumed names of joints for the left arm (based on PR2).
void LeftArmJointNames(std::vector<std::string>* names);

// Assumed names of joints for the right arm (based on PR2).
void RightArmJointNames(std::vector<std::string>* names);

class Editor {
 public:
  Editor(const ProgramDb& db, const JointStateReader& joint_state_reader,
         const Visualizer& visualizer);
  void Start();
  void HandleEvent(const rapid_pbd_msgs::EditorEvent& event);
  bool HandleGetEEPose(rapid_pbd_msgs::GetEEPoseRequest& request,
                       rapid_pbd_msgs::GetEEPoseResponse& response);
  bool HandleGetJointAngles(rapid_pbd_msgs::GetJointAnglesRequest& request,
                            rapid_pbd_msgs::GetJointAnglesResponse& response);

 private:
  void HandleUpdate(const rapid_pbd_msgs::EditorEvent& event);

  ProgramDb db_;
  JointStateReader joint_state_reader_;
  Visualizer viz_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_EDITOR_H_
