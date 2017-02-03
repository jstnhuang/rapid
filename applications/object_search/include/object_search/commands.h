#ifndef _OBJECT_SEARCH_COMMANDS_H_
#define _OBJECT_SEARCH_COMMANDS_H_

#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_msgs/Roi3D.h"
#include "rapid_msgs/StaticCloud.h"
#include "tf/transform_listener.h"
#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_line.h"
#include "rapid_utils/command_interface.h"
#include "rapid_viz/scene_viz.h"

#include "object_search/estimators.h"

namespace rapid {
namespace perception {
class PoseEstimator;
class PoseEstimationMatch;
}
}

namespace object_search {
class CaptureRoi;
class Database;  // Forward declaration

class EditScenesCommand : public rapid::utils::CommandInterface {
 public:
  EditScenesCommand(const rapid::utils::CommandLine& scene_cli);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::utils::CommandLine scene_cli_;
};

class ShowSceneCommand : public rapid::utils::CommandInterface {
 public:
  ShowSceneCommand(rapid::db::NameDb* db, rapid::viz::SceneViz* viz);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* db_;
  rapid::viz::SceneViz* viz_;
};

class ListCommand : public rapid::utils::CommandInterface {
 public:
  ListCommand(rapid::db::NameDb* db, const std::string& type);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

  static const char kLandmarks[];
  static const char kScenes[];

 private:
  rapid::db::NameDb* db_;
  std::string type_;
};

class RecordObjectCommand : public rapid::utils::CommandInterface {
 public:
  RecordObjectCommand(Database* db, CaptureRoi* capture);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;
  std::string last_id();
  std::string last_name();
  rapid_msgs::Roi3D last_roi();

 private:
  Database* db_;
  CaptureRoi* capture_;
  std::string last_id_;    // MongoDB ID of most recent object saved.
  std::string last_name_;  // Name of most recent object saved.
  rapid_msgs::Roi3D last_roi_;
};

class RecordSceneCommand : public rapid::utils::CommandInterface {
 public:
  explicit RecordSceneCommand(rapid::db::NameDb* info_db,
                              rapid::db::NameDb* cloud_db);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* info_db_;
  rapid::db::NameDb* cloud_db_;
  tf::TransformListener tf_listener_;
};

class DeleteCommand : public rapid::utils::CommandInterface {
 public:
  DeleteCommand(Database* db, const std::string& name,
                const std::string& description);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  Database* db_;
  std::string name_;
  std::string description_;
};

class UseCommand : public rapid::utils::CommandInterface {
 public:
  UseCommand(Database* db, Estimators* estimators, const std::string& type,
             const ros::Publisher& pub);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  void CropScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene,
                 std::vector<int>* indices);

  Database* db_;
  Estimators* estimators_;
  std::string type_;
  ros::Publisher pub_;
};

class RunCommand : public rapid::utils::CommandInterface {
 public:
  RunCommand(Estimators* estimator, const ros::Publisher& output_pub);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;
  void matches(std::vector<rapid::perception::PoseEstimationMatch>* matches);

 private:
  void UpdateCustomParams();
  void UpdateRansacParams();
  void UpdateGroupingParams();
  Estimators* estimators_;
  std::vector<rapid::perception::PoseEstimationMatch> matches_;
  ros::Publisher output_pub_;
};

class SetDebugCommand : public rapid::utils::CommandInterface {
 public:
  SetDebugCommand(Estimators* estimator);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  Estimators* estimators_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_COMMANDS_H_
