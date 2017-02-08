#ifndef _OBJECT_SEARCH_COMMANDS_H_
#define _OBJECT_SEARCH_COMMANDS_H_

#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_db/name_db.hpp"
#include "rapid_msgs/LandmarkInfo.h"
#include "rapid_msgs/SceneInfo.h"
#include "rapid_msgs/Roi3D.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_utils/command_interface.h"
#include "rapid_utils/command_line.h"
#include "rapid_viz/scene_viz.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

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

// Command that runs an embedded command line interface.
class CliCommand : public rapid::utils::CommandInterface {
 public:
  CliCommand(const rapid::utils::CommandLine& scene_cli,
             const std::string& name, const std::string& description);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::utils::CommandLine cli_;
  std::string name_;
  std::string description_;
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
  ListCommand(rapid::db::NameDb* db, const std::string& type,
              const std::string& name, const std::string& description);
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

class SetLandmarkSceneCommand : public rapid::utils::CommandInterface {
 public:
  SetLandmarkSceneCommand(rapid::db::NameDb* scene_cloud_db,
                          std::string* scene_name,
                          sensor_msgs::PointCloud2::Ptr landmark_scene,
                          rapid::viz::SceneViz* viz);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* scene_cloud_db_;
  std::string* scene_name_;
  sensor_msgs::PointCloud2::Ptr landmark_scene_;
  rapid::viz::SceneViz* viz_;
};

class EditBoxCommand : public rapid::utils::CommandInterface {
 public:
  EditBoxCommand(rapid::perception::Box3DRoiServer* box_server,
                 rapid_msgs::Roi3D* roi);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::perception::Box3DRoiServer* box_server_;
  rapid_msgs::Roi3D* roi_;
};

class SaveLandmarkCommand : public rapid::utils::CommandInterface {
 public:
  SaveLandmarkCommand(rapid::db::NameDb* info_db, rapid::db::NameDb* cloud_db,
                      sensor_msgs::PointCloud2::Ptr landmark_scene,
                      const std::string& landmark_name, std::string* scene_name,
                      rapid_msgs::Roi3D* roi, const std::string& type);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* info_db_;
  rapid::db::NameDb* cloud_db_;
  sensor_msgs::PointCloud2::Ptr landmark_scene_;
  const std::string landmark_name_;
  std::string* scene_name_;
  rapid_msgs::Roi3D* roi_;
  const std::string type_;
};

class EditLandmarkCommand : public rapid::utils::CommandInterface {
 public:
  EditLandmarkCommand(rapid::db::NameDb* landmark_info_db,
                      rapid::db::NameDb* landmark_cloud_db,
                      rapid::db::NameDb* scene_info_db,
                      rapid::db::NameDb* scene_cloud_db,
                      rapid::perception::Box3DRoiServer* box_server,
                      rapid::viz::SceneViz* scene_viz, const std::string& type);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

  static const char kCreate[];
  static const char kEdit[];

 private:
  rapid::db::NameDb* landmark_info_db_;
  rapid::db::NameDb* landmark_cloud_db_;
  rapid::db::NameDb* scene_info_db_;
  rapid::db::NameDb* scene_cloud_db_;
  rapid::perception::Box3DRoiServer* box_server_;
  sensor_msgs::PointCloud2::Ptr landmark_scene_;
  rapid::viz::SceneViz* scene_viz_;
  std::string type_;
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
  DeleteCommand(rapid::db::NameDb* info_db, rapid::db::NameDb* cloud_db,
                const std::string& type);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* info_db_;
  rapid::db::NameDb* cloud_db_;
  std::string type_;
};

struct PoseEstimatorInput {
  rapid_msgs::LandmarkInfo landmark;
  sensor_msgs::PointCloud2 landmark_cloud;
  rapid_msgs::SceneInfo scene;
  sensor_msgs::PointCloud2 scene_cloud;
};

class SetInputLandmarkCommand : public rapid::utils::CommandInterface {
 public:
  SetInputLandmarkCommand(rapid::db::NameDb* info_db,
                          rapid::db::NameDb* cloud_db,
                          const ros::Publisher& pub, PoseEstimatorInput* input);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* info_db_;
  rapid::db::NameDb* cloud_db_;
  ros::Publisher pub_;
  PoseEstimatorInput* input_;
};

class SetInputSceneCommand : public rapid::utils::CommandInterface {
 public:
  SetInputSceneCommand(rapid::db::NameDb* info_db, rapid::db::NameDb* cloud_db,
                       const rapid::viz::SceneViz& viz,
                       PoseEstimatorInput* input);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* info_db_;
  rapid::db::NameDb* cloud_db_;
  rapid::viz::SceneViz viz_;
  PoseEstimatorInput* input_;
};

class RunCommand : public rapid::utils::CommandInterface {
 public:
  RunCommand(Estimators* estimator, PoseEstimatorInput* input,
             const ros::Publisher& output_pub);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;
  void matches(std::vector<rapid::perception::PoseEstimationMatch>* matches);

 private:
  void CropScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped);
  void Downsample(const double leaf_size,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
  Estimators* estimators_;
  PoseEstimatorInput* input_;
  ros::Publisher output_pub_;
  std::vector<rapid::perception::PoseEstimationMatch> matches_;
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
