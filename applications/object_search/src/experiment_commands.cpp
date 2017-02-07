#include "object_search/experiment_commands.h"

#include <iostream>

#include "boost/algorithm/string.hpp"
#include "object_search_msgs/Label.h"
#include "object_search_msgs/Task.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_msgs/LandmarkInfo.h"
#include "rapid_msgs/SceneInfo.h"
#include "rapid_viz/cloud_poser.h"
#include "rapid_viz/publish.h"

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

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_landmarks(
  //    new pcl::PointCloud<pcl::PointXYZRGB>);
  markers_.clear();
  for (size_t i = 0; i < task.labels.size(); ++i) {
    const object_search_msgs::Label& label = task.labels[i];
    // sensor_msgs::PointCloud2 landmark_cloud;
    // if (!dbs_.landmark_cloud_db->Get(label.landmark_name, &landmark_cloud)) {
    //  ROS_ERROR("Landmark cloud \"%s\" not found.",
    //            label.landmark_name.c_str());
    //  continue;
    //}

    rapid_msgs::LandmarkInfo landmark_info;
    if (!dbs_.landmark_db->Get(label.landmark_name, &landmark_info)) {
      ROS_ERROR("Landmark info \"%s\" not found.", label.landmark_name.c_str());
      continue;
    }

    // stf::Graph graph;
    // stf::Transform
    // original_transform(landmark_info.roi.transform.translation,
    //                                  landmark_info.roi.transform.rotation);
    // graph.Add("original", stf::RefFrame("base_link"), original_transform);
    // graph.Add("current", stf::RefFrame("base_link"), label.pose);
    // stf::Transform original_to_current;
    // graph.ComputeMapping(stf::From("original"), stf::To("current"),
    //                     &original_to_current);
    // sensor_msgs::PointCloud2 out;
    // pcl_ros::transformPointCloud(original_to_current.matrix(),
    // landmark_cloud,
    //                             out);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_landmark(
    //    new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::moveFromROSMsg(out, *pcl_landmark);
    //*all_landmarks += *pcl_landmark;

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "base_link";
    ps.pose = label.pose;
    markers_.push_back(rapid::viz::Marker::OutlineBox(
        &rapid_marker_pub_, ps, landmark_info.roi.dimensions));
    markers_.back().SetNamespace(label.name);
    markers_.back().Publish();
  }
}

void TaskViz::Clear() {
  scene_viz_.Clear();
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
    task_viz_.Publish(*task_);
  }
  task_viz_.Clear();
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

  // Get the pose
  rapid::viz::CloudPoser poser(cloud, landmark_pub_, "cloud_poser");

  std::cout << "Adjust the pose in rviz, then press enter to save: ";
  poser.Start();
  std::string line("");
  std::getline(std::cin, line);

  label_->pose = poser.pose();
  poser.Stop();

  stf::Transform t(label_->pose);
  std::cout << "Pose saved with transform:" << std::endl
            << t.matrix() << std::endl;

  label_->task_name = task_->name;

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
}  // namespace object_search