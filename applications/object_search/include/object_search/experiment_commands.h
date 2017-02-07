#ifndef _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_
#define _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_

#include <string>
#include <vector>

#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_interface.h"
#include "rapid_viz/landmark_viz.h"
#include "rapid_viz/scene_viz.h"

namespace object_search {
struct ExperimentDbs {
  rapid::db::NameDb* task_db;
  rapid::db::NameDb* label_db;
  rapid::db::NameDb* scene_db;
  rapid::db::NameDb* scene_cloud_db;
  rapid::db::NameDb* landmark_db;
  rapid::db::NameDb* landmark_cloud_db;
};

struct ExperimentVizs {
  rapid::viz::SceneViz* scene_viz;
  rapid::viz::LandmarkViz* landmark_viz;
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

class ShowTask : public rapid::utils::CommandInterface {
 public:
  ShowTask(const ExperimentDbs& dbs, const ExperimentVizs& vizs);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
  ExperimentVizs vizs_;
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
  EditTask(const ExperimentDbs& dbs);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  ExperimentDbs dbs_;
};

}  // namespace object_search

#endif  // _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_
