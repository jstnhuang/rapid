#include "rapid_pbd/editor.h"

#include <exception>
#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/GetEEPose.h"
#include "rapid_pbd_msgs/GetJointAngles.h"
#include "rapid_pbd_msgs/GetTorsoPose.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/SegmentSurfacesGoal.h"
#include "rapid_pbd_msgs/Step.h"
#include "tf/transform_listener.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/visualizer.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
Editor::Editor(const ProgramDb& db, const SceneDb& scene_db,
               const JointStateReader& joint_state_reader,
               const Visualizer& visualizer, ActionClients* action_clients,
               const RobotConfig& robot_config)
    : db_(db),
      scene_db_(scene_db),
      joint_state_reader_(joint_state_reader),
      viz_(visualizer),
      action_clients_(action_clients),
      robot_config_(robot_config),
      tf_listener_() {}

void Editor::Start() {
  db_.Start();
  joint_state_reader_.Start();
  viz_.Init();
}

void Editor::HandleEvent(const msgs::EditorEvent& event) {
  try {
    if (event.type == msgs::EditorEvent::CREATE) {
      Create(event.program_info.name);
    } else if (event.type == msgs::EditorEvent::UPDATE) {
      Update(event.program_info.db_id, event.program);
    } else if (event.type == msgs::EditorEvent::DELETE) {
      Delete(event.program_info.db_id);
    } else if (event.type == msgs::EditorEvent::ADD_STEP) {
      AddStep(event.program_info.db_id);
    } else if (event.type == msgs::EditorEvent::DELETE_STEP) {
      DeleteStep(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::ADD_ACTION) {
      AddAction(event.program_info.db_id, event.step_num, event.action);
    } else if (event.type == msgs::EditorEvent::DELETE_ACTION) {
      DeleteAction(event.program_info.db_id, event.step_num, event.action_num);
    } else if (event.type == msgs::EditorEvent::VIEW_STEP) {
      ViewStep(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::DETECT_SURFACE_OBJECTS) {
      DetectSurfaceObjects(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::GET_JOINT_VALUES) {
      GetJointValues(event.program_info.db_id, event.step_num, event.action_num,
                     event.action.actuator_group);
    } else if (event.type == msgs::EditorEvent::GET_POSE) {
      GetPose(event.program_info.db_id, event.step_num, event.action_num,
              event.action.actuator_group, event.action.landmark);
    } else {
      ROS_ERROR("Unknown event type \"%s\"", event.type.c_str());
    }
  } catch (const std::exception& ex) {
    ROS_ERROR("Unhandled exception for event %s: %s", event.type.c_str(),
              ex.what());
  }
}

void Editor::Create(const std::string& name) {
  msgs::Program program;
  program.name = name;
  joint_state_reader_.ToMsg(&program.start_joint_state);
  std::string id = db_.Insert(program);
  viz_.Publish(id, 0);
}

void Editor::Update(const std::string& db_id, const msgs::Program& program) {
  db_.Update(db_id, program);
  viz_.Update(db_id, program);
}

void Editor::Delete(const std::string& db_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete program ID \"%s\"", db_id.c_str());
    return;
  }
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const msgs::Step& step = program.steps[i];
    DeleteScene(step.scene_id);
  }

  db_.Delete(db_id);
  viz_.StopPublishing(db_id);
}

void Editor::AddStep(const std::string& db_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to add step to program ID \"%s\"", db_id.c_str());
    return;
  }
  msgs::Step step;
  program.steps.push_back(step);
  Update(db_id, program);
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
  DeleteScene(program.steps[step_id].scene_id);
  program.steps.erase(program.steps.begin() + step_id);
  Update(db_id, program);
}

void Editor::AddAction(const std::string& db_id, size_t step_id,
                       rapid_pbd_msgs::Action action) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to add action to program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to delete action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];

  step->actions.insert(step->actions.begin(), action);
  Update(db_id, program);
}

void Editor::DeleteAction(const std::string& db_id, size_t step_id,
                          size_t action_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete action from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to delete action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  if (action_id >= step->actions.size()) {
    ROS_ERROR(
        "Unable to delete action %ld from step %ld of program \"%s\", which "
        "has %ld actions",
        action_id, step_id, db_id.c_str(), step->actions.size());
    return;
  }

  // Clean up the scene and landmarks
  const msgs::Action& action = step->actions[action_id];
  if (action.type == msgs::Action::DETECT_TABLETOP_OBJECTS) {
    DeleteScene(step->scene_id);
    step->scene_id = "";
    DeleteLandmarks(msgs::Landmark::SURFACE_BOX, step);
  } else if (action.type == msgs::Action::FIND_CUSTOM_LANDMARK) {
    DeleteScene(step->scene_id);
    step->scene_id = "";
    DeleteLandmarks(msgs::Landmark::CUSTOM_LANDMARK, step);
  }

  step->actions.erase(step->actions.begin() + action_id);
  Update(db_id, program);
}

void Editor::ViewStep(const std::string& db_id, size_t step_id) {
  db_.StartPublishingProgramById(db_id);
  viz_.Publish(db_id, step_id);
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
  DeleteScene(program.steps[step_id].scene_id);
  program.steps[step_id].scene_id = result->cloud_db_id;
  DeleteLandmarks(msgs::Landmark::SURFACE_BOX, &program.steps[step_id]);
  program.steps[step_id].landmarks.insert(
      program.steps[step_id].landmarks.end(), result->landmarks.begin(),
      result->landmarks.end());
  Update(db_id, program);
}

void Editor::GetJointValues(const std::string& db_id, size_t step_id,
                            size_t action_id,
                            const std::string& actuator_group) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to update action from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to update action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  if (action_id >= step->actions.size()) {
    ROS_ERROR(
        "Unable to update action %ld from step %ld of program \"%s\", which "
        "has %ld actions",
        action_id, step_id, db_id.c_str(), step->actions.size());
    return;
  }

  msgs::Action* action = &step->actions[action_id];
  action->actuator_group = actuator_group;
  action->joint_trajectory.joint_names.clear();
  robot_config_.joints_for_group(actuator_group,
                                 &action->joint_trajectory.joint_names);
  if (action->joint_trajectory.joint_names.size() == 0) {
    ROS_ERROR("Can't get joint angles for actuator group \"%s\"",
              action->actuator_group.c_str());
    return;
  }
  action->joint_trajectory.points.resize(1);
  action->joint_trajectory.points[0].positions.clear();

  for (size_t i = 0; i < action->joint_trajectory.joint_names.size(); ++i) {
    const std::string& name = action->joint_trajectory.joint_names[i];
    double pos = joint_state_reader_.get_position(name);
    if (pos == kNoJointValue) {
      ROS_ERROR("Could not get angle for joint \"%s\"", name.c_str());
      action->joint_trajectory.points[0].positions.push_back(0);
    } else {
      action->joint_trajectory.points[0].positions.push_back(pos);
    }
  }

  // Fill in default time
  if (action->joint_trajectory.points[0].time_from_start.isZero()) {
    action->joint_trajectory.points[0].time_from_start.sec = 3;
  }

  Update(db_id, program);
}
void Editor::GetPose(const std::string& db_id, size_t step_id, size_t action_id,
                     const std::string& actuator_group,
                     const rapid_pbd_msgs::Landmark& landmark) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to get action from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  if (action_id >= step->actions.size()) {
    ROS_ERROR(
        "Unable to get action %ld from step %ld of program \"%s\", which "
        "has %ld actions",
        action_id, step_id, db_id.c_str(), step->actions.size());
    return;
  }

  msgs::Action* action = &step->actions[action_id];

  action->actuator_group = actuator_group;

  if (landmark.type == "") {
    action->landmark.type = msgs::Landmark::TF_FRAME;
    action->landmark.frame_id = robot_config_.torso_link();
  } else {
    action->landmark = landmark;
  }

  // Get transform from landmark to end-effector.
  transform_graph::Graph graph;

  // Get transform of end-effector relative to base.
  tf::StampedTransform transform;
  try {
    std::string ee_frame = robot_config_.ee_frame_for_group(actuator_group);
    if (ee_frame == "") {
      ROS_ERROR("Unable to get pose for actuator group: \"%s\"",
                actuator_group.c_str());
    }
    tf_listener_.lookupTransform(robot_config_.base_link(), ee_frame,
                                 ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  graph.Add("end effector",
            transform_graph::RefFrame(robot_config_.base_link()), transform);

  // Get transform of landmark relative to base.
  if (action->landmark.type == msgs::Landmark::TF_FRAME) {
    tf::StampedTransform landmark_transform;
    try {
      tf_listener_.lookupTransform(robot_config_.base_link(),
                                   action->landmark.frame_id, ros::Time(0),
                                   landmark_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    graph.Add("landmark", transform_graph::RefFrame(robot_config_.base_link()),
              landmark_transform);
  } else {
    ROS_ERROR("Unsupported landmark type \"%s\"",
              action->landmark.type.c_str());
    return;
  }

  transform_graph::Transform ee_in_landmark;
  graph.ComputeDescription(transform_graph::LocalFrame("end effector"),
                           transform_graph::RefFrame("landmark"),
                           &ee_in_landmark);
  ee_in_landmark.ToPose(&action->pose);

  Update(db_id, program);
}

void Editor::DeleteScene(const std::string& scene_id) {
  if (scene_id == "") {
    return;
  }
  bool success = scene_db_.Delete(scene_id);
  if (!success) {
    ROS_ERROR("Failed to delete scene ID: \"%s\"", scene_id.c_str());
  }
}

void Editor::DeleteLandmarks(const std::string& landmark_type,
                             rapid_pbd_msgs::Step* step) {
  std::vector<msgs::Landmark> cleaned;
  for (size_t i = 0; i < step->landmarks.size(); ++i) {
    const msgs::Landmark& landmark = step->landmarks[i];
    if (landmark.type != landmark_type) {
      cleaned.push_back(landmark);
    }
  }
  step->landmarks = cleaned;
}
}  // namespace pbd
}  // namespace rapid
