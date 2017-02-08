#ifndef _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_
#define _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_

#include <string>
#include <vector>

#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_interface.h"
#include "rapid_utils/command_line.h"
#include "rapid_viz/landmark_viz.h"
#include "rapid_viz/markers.h"
#include "rapid_viz/scene_viz.h"
#include "visualization_msgs/Marker.h"

#include "object_search/experiment.h"
#include "object_search_msgs/Task.h"

namespace object_search {
struct ExperimentVizs {
  rapid::viz::SceneViz* scene_viz;
  rapid::viz::LandmarkViz* landmark_viz;
};

class TaskViz {
 public:
  TaskViz(const ExperimentDbs dbs, const ros::Publisher& scene_pub,
          const ros::Publisher& landmark_pub, const ros::Publisher& marker_pub);
  // Publish the latest visualization.
  void Publish(const object_search_msgs::Task& task);
  void Clear();

 private:
  ExperimentDbs dbs_;
  ros::Publisher scene_pub_;
  ros::Publisher landmark_pub_;
  ros::Publisher marker_pub_;
  std::vector<rapid::viz::Marker> markers_;

  rapid::viz::SceneViz scene_viz_;
  rapid_ros::Publisher<visualization_msgs::Marker> rapid_marker_pub_;
};

class ListTasks : public rapid::utils::CommandInterface {
 public:
  ListTasks(rapid::db::NameDb* db);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* db_;
};

class DeleteTask : public rapid::utils::CommandInterface {
 public:
  DeleteTask(rapid::db::NameDb* db);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* db_;
};

class CreateTask : public rapid::utils::CommandInterface {
 public:
  CreateTask(const ExperimentDbs& dbs);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
};

class EditTask : public rapid::utils::CommandInterface {
 public:
  EditTask(const ExperimentDbs& dbs, const TaskViz& task_viz,
           rapid::utils::CommandLine* task_cli, object_search_msgs::Task* task);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
  TaskViz task_viz_;
  rapid::utils::CommandLine* task_cli_;
  object_search_msgs::Task* task_;
};

class SetTaskScene : public rapid::utils::CommandInterface {
 public:
  SetTaskScene(const ExperimentDbs& dbs, object_search_msgs::Task* task);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
  object_search_msgs::Task* task_;
};

class SetLabelLandmark : public rapid::utils::CommandInterface {
 public:
  SetLabelLandmark(const ExperimentDbs& dbs, object_search_msgs::Task* task,
                   object_search_msgs::Label* label);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
  object_search_msgs::Task* task_;
  object_search_msgs::Label* label_;
};

class ListLabels : public rapid::utils::CommandInterface {
 public:
  ListLabels(object_search_msgs::Task* task);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  object_search_msgs::Task* task_;
};

// This is a copy of ListCommand. For some reason MongoDB dies when you link
// with object_search_commands.
class ListLandmarksOrScenes : public rapid::utils::CommandInterface {
 public:
  ListLandmarksOrScenes(rapid::db::NameDb* db, const std::string& type,
                        const std::string& name,
                        const std::string& description);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

  static const char kLandmarks[];
  static const char kScenes[];

 private:
  rapid::db::NameDb* db_;
  std::string type_;
  std::string name_;
  std::string description_;
};

class AddLabel : public rapid::utils::CommandInterface {
 public:
  AddLabel(const ExperimentDbs& dbs, const ros::Publisher& landmark_pub,
           const ros::Publisher& marker_pub, object_search_msgs::Task* task,
           object_search_msgs::Label* label);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
  ros::Publisher landmark_pub_;
  ros::Publisher marker_pub_;
  object_search_msgs::Task* task_;
  object_search_msgs::Label* label_;
};

class DeleteLabel : public rapid::utils::CommandInterface {
 public:
  DeleteLabel(const ExperimentDbs& dbs, object_search_msgs::Task* task);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
  object_search_msgs::Task* task_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_
