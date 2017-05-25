#include "object_search/experiment_commands.h"

#include <iostream>

#include "boost/algorithm/string.hpp"
#include "object_search_msgs/Label.h"
#include "object_search_msgs/Task.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_msgs/LandmarkInfo.h"
#include "rapid_msgs/SceneInfo.h"
#include "rapid_perception/conversions.h"
#include "rapid_viz/cloud_poser.h"
#include "rapid_viz/point_cloud.h"
#include "rapid_viz/publish.h"
#include "readline/readline.h"

using pcl::PointXYZRGB;
using pcl::PointCloud;
typedef PointCloud<PointXYZRGB> PointCloudC;

namespace object_search {

TaskViz::TaskViz(const ExperimentDbs dbs, const ros::Publisher& scene_pub,
                 const ros::Publisher& landmark_pub,
                 const ros::Publisher& marker_pub)
    : dbs_(dbs),
      scene_pub_(scene_pub),
      landmark_pub_(landmark_pub),
      marker_pub_(marker_pub),
      scene_viz_(scene_pub_),
      rapid_marker_pub_(marker_pub_) {}

void TaskViz::Publish(const object_search_msgs::Task& task) {
  if (task.scene_name != "") {
    sensor_msgs::PointCloud2 scene_cloud;
    bool success = dbs_.scene_cloud_db->Get(task.scene_name, &scene_cloud);
    if (!success) {
      ROS_WARN("Scene cloud \"%s\" not found.", task.scene_name.c_str());
    } else {
      scene_viz_.set_scene(scene_cloud);
    }
  } else {
    ROS_INFO("Task \"%s\" does not have a scene yet.", task.name.c_str());
  }

  pcl::PointCloud<PointXYZRGB>::Ptr all_landmarks(
      new pcl::PointCloud<PointXYZRGB>);
  all_landmarks->header.frame_id = "base_link";
  markers_.clear();
  for (size_t i = 0; i < task.labels.size(); ++i) {
    const object_search_msgs::Label& label = task.labels[i];
    if (!label.exists) {
      continue;
    }
    sensor_msgs::PointCloud2 landmark_cloud;
    if (!dbs_.landmark_cloud_db->Get(label.landmark_name, &landmark_cloud)) {
      ROS_ERROR("Landmark cloud \"%s\" not found.",
                label.landmark_name.c_str());
      continue;
    }

    rapid_msgs::LandmarkInfo landmark_info;
    if (!dbs_.landmark_db->Get(label.landmark_name, &landmark_info)) {
      ROS_ERROR("Landmark info \"%s\" not found.", label.landmark_name.c_str());
      continue;
    }

    // Demean point cloud
    pcl::PointCloud<PointXYZRGB>::Ptr pcl_landmark(
        new pcl::PointCloud<PointXYZRGB>);
    pcl::fromROSMsg(landmark_cloud, *pcl_landmark);
    Eigen::Vector4d centroid;
    centroid << landmark_info.roi.transform.translation.x,
        landmark_info.roi.transform.translation.y,
        landmark_info.roi.transform.translation.z, 1;
    pcl::demeanPointCloud(*pcl_landmark, centroid, *pcl_landmark);

    transform_graph::Transform current_transform(label.pose);
    pcl::PointCloud<PointXYZRGB>::Ptr pcl_out(new pcl::PointCloud<PointXYZRGB>);
    pcl::transformPointCloud(*pcl_landmark, *pcl_out,
                             current_transform.matrix());
    rapid::viz::ColorizeRandom(pcl_out);
    *all_landmarks += *pcl_out;

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "base_link";
    ps.pose = label.pose;
    markers_.push_back(rapid::viz::Marker::OutlineBox(
        &rapid_marker_pub_, ps, landmark_info.roi.dimensions));
    markers_.back().SetNamespace(label.name);
    markers_.back().Publish();
  }
  rapid::viz::PublishCloud(landmark_pub_, *all_landmarks);
}

void TaskViz::Clear() {
  scene_viz_.Clear();
  rapid::viz::PublishBlankCloud(landmark_pub_);
  markers_.clear();
}

ListTasks::ListTasks(rapid::db::NameDb* db) : db_(db) {}
void ListTasks::Execute(const std::vector<std::string>& args) {
  std::vector<std::string> names;
  db_->List<object_search_msgs::Task>(&names);

  if (names.size() == 0) {
    std::cout << "None" << std::endl;
  } else {
    for (size_t i = 0; i < names.size(); ++i) {
      std::cout << names[i] << std::endl;
    }
  }
}
std::string ListTasks::name() const { return "list"; }
std::string ListTasks::description() const { return "- List tasks"; }

DeleteTask::DeleteTask(rapid::db::NameDb* db) : db_(db) {}
void DeleteTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No task to delete.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  bool success = db_->Delete<object_search_msgs::Task>(name);
  if (!success) {
    ROS_ERROR("Task \"%s\" not found.", name.c_str());
    return;
  }
}
std::string DeleteTask::name() const { return "delete"; }
std::string DeleteTask::description() const { return "<name> - Delete a task"; }

CreateTask::CreateTask(const ExperimentDbs& dbs) : dbs_(dbs) {}
void CreateTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No name specified.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  object_search_msgs::Task task;
  task.name = name;
  dbs_.task_db->Insert(name, task);

  std::cout << "Created task named \"" << name << "\"." << std::endl;
}
std::string CreateTask::name() const { return "create"; }
std::string CreateTask::description() const { return "<name> - Create a task"; }

EditTask::EditTask(const ExperimentDbs& dbs, const TaskViz& task_viz,
                   rapid::utils::CommandLine* task_cli,
                   object_search_msgs::Task* task)
    : dbs_(dbs), task_viz_(task_viz), task_cli_(task_cli), task_(task) {}
void EditTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No name specified.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  object_search_msgs::Task task;
  bool success = dbs_.task_db->Get(name, &task);
  if (!success) {
    ROS_ERROR("Task \"%s\" not found.", name.c_str());
    return;
  }

  *task_ = task;

  task_viz_.Publish(task);

  while (task_cli_->Next()) {
    task_viz_.Clear();
    task_viz_.Publish(*task_);
  }
}
std::string EditTask::name() const { return "edit"; }
std::string EditTask::description() const { return "<name> - Edit a task"; }

SetTaskScene::SetTaskScene(const ExperimentDbs& dbs,
                           object_search_msgs::Task* task)
    : dbs_(dbs), task_(task) {}
void SetTaskScene::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No name specified.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  sensor_msgs::PointCloud2 cloud;
  bool success = dbs_.scene_cloud_db->Get(name, &cloud);
  if (!success) {
    ROS_ERROR("Scene cloud \"%s\" not found.", name.c_str());
    return;
  }
  task_->scene_name = name;

  success = dbs_.task_db->Update(task_->name, *task_);
  if (!success) {
    ROS_ERROR("Unable to update task \"%s\"!", task_->name.c_str());
    return;
  }
}
std::string SetTaskScene::name() const { return "use scene"; }
std::string SetTaskScene::description() const {
  return "<name> - Set the scene for this task";
}

SetLabelLandmark::SetLabelLandmark(const ExperimentDbs& dbs,
                                   object_search_msgs::Task* task,
                                   object_search_msgs::Label* label)
    : dbs_(dbs), task_(task), label_(label) {}
void SetLabelLandmark::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No name specified.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  rapid_msgs::LandmarkInfo info;
  bool success = dbs_.landmark_db->Get(name, &info);
  if (!success) {
    ROS_ERROR("Landmark \"%s\" not found.", name.c_str());
    return;
  }
  label_->landmark_name = name;
}
std::string SetLabelLandmark::name() const { return "use landmark"; }
std::string SetLabelLandmark::description() const {
  return "<name> - Set the landmark for the next label";
}

ListLabels::ListLabels(object_search_msgs::Task* task) : task_(task) {}
void ListLabels::Execute(const std::vector<std::string>& args) {
  if (task_->labels.size() == 0) {
    std::cout << "None" << std::endl;
    return;
  }
  for (size_t i = 0; i < task_->labels.size(); ++i) {
    const object_search_msgs::Label& label = task_->labels[i];
    std::cout << label.name << std::endl;
  }
}
std::string ListLabels::name() const { return "list"; }
std::string ListLabels::description() const { return "- List labels"; }

ListLandmarksOrScenes::ListLandmarksOrScenes(rapid::db::NameDb* db,
                                             const std::string& type,
                                             const std::string& name,
                                             const std::string& description)
    : db_(db), type_(type), name_(name), description_(description) {}

void ListLandmarksOrScenes::Execute(const std::vector<std::string>& args) {
  std::vector<std::string> names;
  if (type_ == kLandmarks) {
    db_->List<rapid_msgs::LandmarkInfo>(&names);
    std::cout << "Landmarks:" << std::endl;
  } else {
    db_->List<rapid_msgs::SceneInfo>(&names);
    std::cout << "Scenes:" << std::endl;
  }
  if (names.size() == 0) {
    std::cout << "  None." << std::endl;
  }
  for (size_t i = 0; i < names.size(); ++i) {
    std::cout << "  " << names[i] << std::endl;
  }
}

std::string ListLandmarksOrScenes::name() const { return name_; }
std::string ListLandmarksOrScenes::description() const { return description_; }
const char ListLandmarksOrScenes::kLandmarks[] = "landmark";
const char ListLandmarksOrScenes::kScenes[] = "scene";

AddLabel::AddLabel(const ExperimentDbs& dbs, const ros::Publisher& landmark_pub,
                   const ros::Publisher& marker_pub,
                   object_search_msgs::Task* task,
                   object_search_msgs::Label* label)
    : dbs_(dbs),
      landmark_pub_(landmark_pub),
      marker_pub_(marker_pub),
      task_(task),
      label_(label) {}
void AddLabel::Execute(const std::vector<std::string>& args) {
  // Check pre-conditions
  if (args.size() == 0) {
    ROS_ERROR("No name given for the label.");
    return;
  }
  std::string name(boost::algorithm::join(args, " "));
  label_->name = name;

  if (label_->landmark_name == "") {
    ROS_ERROR("Must specify landmark to use.");
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  if (!dbs_.landmark_cloud_db->Get(label_->landmark_name, &cloud)) {
    ROS_ERROR("Could not get landmark cloud \"%s\"",
              label_->landmark_name.c_str());
    return;
  }
  pcl::PointCloud<PointXYZRGB>::Ptr pcl_landmark(
      new pcl::PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(cloud, *pcl_landmark);

  rapid_msgs::LandmarkInfo landmark_info;
  if (!dbs_.landmark_db->Get(label_->landmark_name, &landmark_info)) {
    ROS_ERROR("Could not get landmark info \"%s\"",
              label_->landmark_name.c_str());
    return;
  }

  if (task_->scene_name == "") {
    ROS_ERROR("Must specify a scene for this task.");
    return;
  }

  sensor_msgs::PointCloud2 scene_cloud;
  if (!dbs_.scene_cloud_db->Get(task_->scene_name, &scene_cloud)) {
    ROS_ERROR("Could not get scene cloud \"%s\"", task_->scene_name.c_str());
    return;
  }
  pcl::PointCloud<PointXYZRGB>::Ptr pcl_scene =
      rapid::perception::PclFromRos(scene_cloud);

  // Compute the offset between the center of the point cloud and the center of
  // the box.
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*pcl_landmark, min_pt, max_pt);
  Eigen::Vector4f center = (max_pt + min_pt) / 2;
  Eigen::Vector3d centroid = center.head<3>().cast<double>();
  transform_graph::Orientation identity;

  transform_graph::Graph graph;
  graph.Add("model_cloud", transform_graph::RefFrame("base_link"),
            transform_graph::Transform(centroid, identity));
  graph.Add("model_roi", transform_graph::RefFrame("base_link"),
            transform_graph::Transform(landmark_info.roi.transform.translation,
                                       landmark_info.roi.transform.rotation));
  transform_graph::Transform cloud_to_roi;
  graph.ComputeMapping(transform_graph::From("model_cloud"),
                       transform_graph::To("model_roi"), &cloud_to_roi);

  // Get the pose
  rapid::viz::CloudPoser poser(cloud, landmark_pub_, "cloud_poser");
  Eigen::Matrix4d icp_transform(Eigen::Matrix4d::Identity());
  while (true) {
    poser.Start();
    char* line = readline("Press enter to run ICP, or q to cancel: ");
    if (std::string(line) == "q" || line == NULL) {
      delete line;
      return;
    }

    graph.Add("current_cloud", transform_graph::RefFrame("base_link"),
              poser.pose());
    poser.Stop();

    // Compute the transform to move the model to the current position.
    transform_graph::Transform model_to_base;
    graph.ComputeMapping(transform_graph::From("model_cloud"),
                         transform_graph::To("base_link"), &model_to_base);
    transform_graph::Transform base_to_current;
    graph.ComputeMapping(transform_graph::From("base_link"),
                         transform_graph::To("current_cloud"),
                         &base_to_current);
    Eigen::Matrix4d model_to_current =
        base_to_current.matrix() * model_to_base.matrix();
    PointCloudC::Ptr user_aligned(new PointCloudC);
    pcl::transformPointCloud(*pcl_landmark, *user_aligned,
                             model_to_current.cast<float>());

    // Align with ICP
    PointCloudC::Ptr icp_aligned(new PointCloudC);
    pcl::IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    icp.setInputSource(user_aligned);
    icp.setInputTarget(pcl_scene);
    icp.align(*icp_aligned);
    icp_transform = icp.getFinalTransformation().cast<double>();

    std::cout << "ICP converged: " << icp.hasConverged()
              << ", fitness: " << icp.getFitnessScore() << std::endl;
    rapid::viz::ColorizeRandom(icp_aligned);
    rapid::viz::PublishCloud(landmark_pub_, *icp_aligned);

    line = readline("Press enter to save, or q to try again: ");
    if (std::string(line) == "q") {
      delete line;
      continue;
    } else {
      delete line;
      break;
    }
  }

  // CloudPoser gives the pose of the center of the point cloud, not of the
  // landmark box. Here we find the pose of the center of the landmark box.
  transform_graph::Transform label_pose;
  graph.MapPose(cloud_to_roi, transform_graph::From("base_link"),
                transform_graph::To("current_cloud"), &label_pose);
  label_pose = transform_graph::Transform(icp_transform * label_pose.matrix());
  Eigen::Affine3d affine(label_pose.matrix());
  Eigen::Quaterniond q(affine.rotation());
  label_->pose.orientation.w = q.w();
  label_->pose.orientation.x = q.x();
  label_->pose.orientation.y = q.y();
  label_->pose.orientation.z = q.z();
  label_->pose.position.x = label_pose.matrix()(0, 3);
  label_->pose.position.y = label_pose.matrix()(1, 3);
  label_->pose.position.z = label_pose.matrix()(2, 3);
  std::cout << "Pose saved with transform:" << std::endl
            << transform_graph::Transform(label_->pose).matrix() << std::endl;

  label_->task_name = task_->name;
  label_->exists = true;

  // Save to DB
  task_->labels.push_back(*label_);
  if (!dbs_.task_db->Update(task_->name, *task_)) {
    ROS_ERROR("Failed to update task \"%s\"!", task_->name.c_str());
    return;
  }
}
std::string AddLabel::name() const { return "label"; }
std::string AddLabel::description() const { return "<name> - Add a label"; }

DeleteLabel::DeleteLabel(const ExperimentDbs& dbs,
                         object_search_msgs::Task* task)
    : dbs_(dbs), task_(task) {}
void DeleteLabel::Execute(const std::vector<std::string>& args) {
  if (args.size() == 0) {
    ROS_ERROR("No name given for the label.");
    return;
  }
  std::string name(boost::algorithm::join(args, " "));
  std::vector<object_search_msgs::Label>::iterator it = task_->labels.begin();
  for (; it != task_->labels.end(); ++it) {
    const object_search_msgs::Label& label = *it;
    if (label.name == name) {
      it = task_->labels.erase(it);
      if (it == task_->labels.end()) {
        break;
      }
    }
  }

  if (!dbs_.task_db->Update(task_->name, *task_)) {
    ROS_ERROR("Failed to update task \"%s\"!", task_->name.c_str());
    return;
  }
}
std::string DeleteLabel::name() const { return "delete"; }
std::string DeleteLabel::description() const {
  return "<name> - Delete a label";
}

AddNegativeLabel::AddNegativeLabel(const ExperimentDbs& dbs,
                                   object_search_msgs::Task* task)
    : dbs_(dbs), task_(task) {}

void AddNegativeLabel::Execute(const std::vector<std::string>& args) {
  if (args.size() < 1) {
    ROS_ERROR("No negative landmark given.");
    return;
  }
  std::string name(boost::algorithm::join(args, " "));

  rapid_msgs::LandmarkInfo landmark_info;
  if (!dbs_.landmark_db->Get(name, &landmark_info)) {
    ROS_ERROR("Landmark info \"%s\" not found.", name.c_str());
    return;
  }

  object_search_msgs::Label label;
  label.name = "negative " + name;
  label.task_name = task_->name;
  label.landmark_name = name;
  label.exists = false;

  // Save to DB
  task_->labels.push_back(label);
  if (!dbs_.task_db->Update(task_->name, *task_)) {
    ROS_ERROR("Failed to update task \"%s\"!", task_->name.c_str());
    return;
  }
}

std::string AddNegativeLabel::name() const { return "add negative"; }

std::string AddNegativeLabel::description() const {
  return "<landmark> - Add a landmark that should not be found in the scene.";
}
}  // namespace object_search
