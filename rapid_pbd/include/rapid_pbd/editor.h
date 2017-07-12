#ifndef _RAPID_PBD_EDITOR_H_
#define _RAPID_PBD_EDITOR_H_

#include <string>
#include <vector>

#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/GetEEPose.h"
#include "rapid_pbd_msgs/GetJointAngles.h"
#include "rapid_pbd_msgs/GetTorsoPose.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/visualizer.h"

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
  Editor(const ProgramDb& db, const SceneDb& scene_db,
         const JointStateReader& joint_state_reader,
         const Visualizer& visualizer, ActionClients* action_clients);
  void Start();
  void HandleEvent(const rapid_pbd_msgs::EditorEvent& event);
  bool HandleGetEEPose(rapid_pbd_msgs::GetEEPoseRequest& request,
                       rapid_pbd_msgs::GetEEPoseResponse& response);
  bool HandleGetTorsoPose(rapid_pbd_msgs::GetTorsoPoseRequest& request,
                          rapid_pbd_msgs::GetTorsoPoseResponse& response);
  bool HandleGetJointAngles(rapid_pbd_msgs::GetJointAnglesRequest& request,
                            rapid_pbd_msgs::GetJointAnglesResponse& response);

 private:
  void Create(const std::string& name);
  void Update(const std::string& db_id, const rapid_pbd_msgs::Program& program);
  void Delete(const std::string& db_id);
  void AddStep(const std::string& db_id);
  void DeleteStep(const std::string& db_id, size_t step_id);
  void AddAction(const std::string& db_id, size_t step_id,
                 const rapid_pbd_msgs::Action& action);
  void DeleteAction(const std::string& db_id, size_t step_id, size_t action_id);
  void ViewStep(const std::string& db_id, size_t step_id);
  void DetectSurfaceObjects(const std::string& db_id, size_t step_id);

  ProgramDb db_;
  SceneDb scene_db_;
  JointStateReader joint_state_reader_;
  Visualizer viz_;
  ActionClients* action_clients_;
  tf::TransformListener tf_listener_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_EDITOR_H_
