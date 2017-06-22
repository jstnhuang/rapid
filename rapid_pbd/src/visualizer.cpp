#include "rapid_pbd/visualizer.h"

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "robot_markers/builder.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"

namespace msgs = rapid_pbd_msgs;
using sensor_msgs::PointCloud2;
using visualization_msgs::MarkerArray;

namespace rapid {
namespace pbd {
Visualizer::Visualizer(const ProgramDb& db, const SceneDb& scene_db,
                       const robot_markers::Builder& marker_builder)
    : db_(db),
      scene_db_(scene_db),
      marker_builder_(marker_builder),
      step_vizs_(),
      nh_() {}

void Visualizer::Init() {
  marker_builder_.Init();
  marker_builder_.SetNamespace("robot");
  marker_builder_.SetFrameId("base_link");
}

void Visualizer::Publish(const std::string& program_id, int step_num) {
  msgs::Program program;
  if (program_id == "" || !db_.Get(program_id, &program)) {
    return;
  }
  size_t step_id = static_cast<size_t>(step_num);

  // Create the publisher if it doesn't exist.
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    step_vizs_[program_id].robot_pub =
        nh_.advertise<MarkerArray>("robot/" + program_id, 10, true);
    step_vizs_[program_id].scene_pub =
        nh_.advertise<PointCloud2>("scene/" + program_id, 10, true);
  }

  // Publish the robot visualization
  step_vizs_[program_id].step_id = step_id;

  MarkerArray robot_markers;
  GetRobotMarker(program, step_id, &robot_markers);
  step_vizs_[program_id].robot_pub.publish(robot_markers);

  PointCloud2 scene;
  bool got_scene = GetScene(program, step_id, &scene);
  if (got_scene) {
    step_vizs_[program_id].scene_pub.publish(scene);
  } else {
    pcl::PointCloud<pcl::PointXYZRGB> blank;
    pcl::PointXYZRGB pt;
    blank.points.push_back(pt);
    pcl::toROSMsg(blank, scene);
    scene.header.frame_id = "base_link";
    step_vizs_[program_id].scene_pub.publish(scene);
  }
}

void Visualizer::Update(const std::string& program_id,
                        const rapid_pbd_msgs::Program& program) {
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    return;
  }
  Publish(program_id, step_vizs_[program_id].step_id);

  // MarkerArray robot_markers;
  // GetRobotMarker(program, step_vizs_[program_id].step_id, &robot_markers);
  // step_vizs_[program_id].robot_pub.publish(robot_markers);
}

void Visualizer::StopPublishing(const std::string& program_id) {
  if (step_vizs_.find(program_id) != step_vizs_.end()) {
    step_vizs_.erase(program_id);
  }
}

bool Visualizer::GetRobotMarker(const msgs::Program& program, size_t step_id,
                                MarkerArray* robot_markers) {
  // Update the joint state for each step.
  JointState current(program.start_joint_state);
  for (size_t step_i = 0; step_i <= step_id; ++step_i) {
    if (step_i >= program.steps.size()) {
      break;
    }
    const msgs::Step& step = program.steps[step_i];
    for (size_t action_i = 0; action_i < step.actions.size(); ++action_i) {
      const msgs::Action& action = step.actions[action_i];
      if (action.type == msgs::Action::ACTUATE_GRIPPER) {
        // TODO: fill this in.
      } else if (action.type == msgs::Action::MOVE_TO_JOINT_GOAL) {
        const trajectory_msgs::JointTrajectory& trajectory =
            action.joint_trajectory;
        if (trajectory.points.size() == 0) {
          continue;
        }
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
          const std::string& name = trajectory.joint_names[i];
          double position = trajectory.points[0].positions[i];
          current.SetPosition(name, position);
        }
      } else if (action.type == msgs::Action::MOVE_TO_CARTESIAN_GOAL) {
        // TODO: fill this in.
      }
    }
  }
  std::map<std::string, double> joint_positions;
  current.ToMap(&joint_positions);

  marker_builder_.SetJointPositions(joint_positions);
  marker_builder_.Build(robot_markers);
  return true;
}

bool Visualizer::GetScene(const rapid_pbd_msgs::Program& program,
                          size_t step_num, sensor_msgs::PointCloud2* scene) {
  if (step_num >= program.steps.size()) {
    ROS_ERROR(
        "Cannot get scene for step %ld of program %s, which has %ld steps.",
        step_num, program.name.c_str(), program.steps.size());
    return false;
  }
  // size_t will wrap around
  for (size_t i = step_num; i < program.steps.size(); --i) {
    const msgs::Step& step = program.steps[i];
    if (step.scene_id != "") {
      bool success = scene_db_.Get(step.scene_id, scene);
      if (!success) {
        ROS_ERROR("Failed to get scene ID: \"%s\"", step.scene_id.c_str());
        return false;
      }
      return true;
    }
  }
  return false;
}
}  // namespace pbd
}  // namespace rapid
