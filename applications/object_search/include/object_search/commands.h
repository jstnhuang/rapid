#ifndef _OBJECT_SEARCH_COMMANDS_H_
#define _OBJECT_SEARCH_COMMANDS_H_

#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_msgs/Roi3D.h"
#include "rapid_msgs/StaticCloud.h"
#include "tf/transform_listener.h"

namespace rapid {
namespace perception {
class PoseEstimator;
class PoseEstimationMatch;
}
}

namespace object_search {
class CaptureRoi;
class Database;  // Forward declaration

class Command {
 public:
  virtual ~Command() {}
  virtual void Execute(std::vector<std::string>& args) = 0;
};

class ListCommand : public Command {
 public:
  ListCommand(Database* db, const std::string& type);
  void Execute(std::vector<std::string>& args);

 private:
  Database* db_;
  std::string type_;
};

class RecordObjectCommand : public Command {
 public:
  RecordObjectCommand(Database* db, CaptureRoi* capture);
  void Execute(std::vector<std::string>& args);
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

class RecordSceneCommand : public Command {
 public:
  explicit RecordSceneCommand(Database* db);
  void Execute(std::vector<std::string>& args);

 private:
  Database* db_;
  tf::TransformListener tf_listener_;
};

class DeleteCommand : public Command {
 public:
  DeleteCommand(Database* db);
  void Execute(std::vector<std::string>& args);

 private:
  Database* db_;
};

class UseCommand : public Command {
 public:
  UseCommand(Database* db, rapid::perception::PoseEstimator* estimator,
             const std::string& type);
  void Execute(std::vector<std::string>& args);

 private:
  void CropScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene,
                 std::vector<int>* indices);

  Database* db_;
  rapid::perception::PoseEstimator* estimator_;
  std::string type_;
};

class RunCommand : public Command {
 public:
  RunCommand(rapid::perception::PoseEstimator* estimator);
  void Execute(std::vector<std::string>& args);
  void matches(std::vector<rapid::perception::PoseEstimationMatch>* matches);

 private:
  void UpdateParams();
  rapid::perception::PoseEstimator* estimator_;
  std::vector<rapid::perception::PoseEstimationMatch> matches_;
};

class SetDebugCommand : public Command {
 public:
  SetDebugCommand(rapid::perception::PoseEstimator* estimator);
  void Execute(std::vector<std::string>& args);

 private:
  rapid::perception::PoseEstimator* estimator_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_COMMANDS_H_
