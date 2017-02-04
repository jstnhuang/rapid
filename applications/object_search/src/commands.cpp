#include "object_search/commands.h"

#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "boost/algorithm/string.hpp"
#include "pcl/common/time.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_db/name_db.hpp"
#include "rapid_msgs/LandmarkInfo.h"
#include "rapid_msgs/SceneInfo.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/pose_estimation_match.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_utils/command_line.h"
#include "rapid_viz/publish.h"
#include "rapid_viz/scene_viz.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#include "object_search/capture_roi.h"
#include "object_search/cloud_database.h"
#include "object_search/estimators.h"

using pcl::PointCloud;
using pcl::PointXYZRGB;
using sensor_msgs::PointCloud2;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using rapid_msgs::StaticCloud;
using rapid::db::NameDb;
using rapid::perception::PoseEstimationMatch;
using rapid::perception::GroupingPoseEstimator;
using rapid::viz::SceneViz;

namespace object_search {
CliCommand::CliCommand(const rapid::utils::CommandLine& cli, const string& name,
		       const string& description)
    : cli_(cli), name_(name), description_(description) {}
void CliCommand::Execute(const vector<string>& args) {
  while (cli_.Next()) {
  }
}
string CliCommand::name() const { return name_; }
string CliCommand::description() const { return description_; }

ShowSceneCommand::ShowSceneCommand(NameDb* db, SceneViz* viz)
    : db_(db), viz_(viz) {}

void ShowSceneCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    ROS_ERROR("Error: provide a name of a scene to show.");
    return;
  }
  string name = boost::algorithm::join(args, " ");
  PointCloud2 cloud;
  bool success = db_->Get(name, &cloud);
  if (!success) {
    ROS_ERROR("Error: scene %s was not found.", name.c_str());
    return;
  }

  viz_->set_scene(cloud);
}
string ShowSceneCommand::name() const { return "show"; }
string ShowSceneCommand::description() const {
  return "<name> - Show a scene in the visualization";
}

ListCommand::ListCommand(NameDb* db, const string& type, const string& name,
			 const string& description)
    : db_(db), type_(type), name_(name), description_(description) {}

void ListCommand::Execute(const vector<string>& args) {
  vector<string> names;
  if (type_ == kLandmarks) {
    db_->List<rapid_msgs::LandmarkInfo>(&names);
    cout << "Landmarks:" << endl;
  } else {
    db_->List<rapid_msgs::SceneInfo>(&names);
    cout << "Scenes:" << endl;
  }
  if (names.size() == 0) {
    cout << "  None." << endl;
  }
  for (size_t i = 0; i < names.size(); ++i) {
    cout << "  " << names[i] << endl;
  }
}

string ListCommand::name() const { return name_; }
string ListCommand::description() const { return description_; }
const char ListCommand::kLandmarks[] = "landmark";
const char ListCommand::kScenes[] = "scene";

RecordObjectCommand::RecordObjectCommand(Database* db, CaptureRoi* capture)
    : db_(db), capture_(capture), last_id_(""), last_name_("") {}

void RecordObjectCommand::Execute(const vector<string>& args) {
  last_id_ = "";    // Reset ID
  last_name_ = "";  // Reset ID
  rapid_msgs::Roi3D roi;
  last_roi_ = roi;

  // Start server and wait for input
  capture_->ShowMarker();
  cout << "Adjust the ROI marker in rviz." << endl;

  string name("");
  string input("");
  if (args.size() > 0 && args[0] != "") {
    name = boost::algorithm::join(args, " ");
    cout << "Type \"save\" to save or \"cancel\" to cancel: ";
    std::getline(std::cin, input);
  } else {
    cout << "Give this landmark a name, or type \"cancel\" to cancel: ";
    std::getline(std::cin, input);
    name = input;
  }
  capture_->HideMarker();
  if (input == "cancel") {
    return;
  }

  // Read cloud and saved region
  PointCloud2::ConstPtr cloud_in =
      ros::topic::waitForMessage<PointCloud2>("cloud_in", ros::Duration(10));
  capture_->set_cloud(cloud_in);
  StaticCloud static_cloud;
  capture_->Capture(&static_cloud.cloud);

  // Get transform
  static_cloud.parent_frame_id = capture_->base_frame();
  tf::transformTFToMsg(capture_->cloud_to_base().inverse(),
		       static_cloud.base_to_camera);
  static_cloud.roi = capture_->roi();
  last_roi_ = static_cloud.roi;

  // Save static cloud
  static_cloud.name = name;
  last_id_ = db_->Save(static_cloud);
  last_name_ = name;
  cout << "Saved " << static_cloud.name << " with ID " << last_id_ << endl;
}
string RecordObjectCommand::name() const { return "record object"; }
string RecordObjectCommand::description() const {
  return "<name> - Save a new object";
}

string RecordObjectCommand::last_id() { return last_id_; }

string RecordObjectCommand::last_name() { return last_name_; }

rapid_msgs::Roi3D RecordObjectCommand::last_roi() { return last_roi_; }

SetLandmarkSceneCommand::SetLandmarkSceneCommand(
    NameDb* scene_cloud_db, string* scene_name, PointCloud2::Ptr landmark_scene,
    SceneViz* viz)
    : scene_cloud_db_(scene_cloud_db),
      scene_name_(scene_name),
      landmark_scene_(landmark_scene),
      viz_(viz) {}

void SetLandmarkSceneCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: must supply scene name." << endl;
    return;
  }
  string name(boost::algorithm::join(args, " "));
  PointCloud2 cloud;
  bool success = scene_cloud_db_->Get(name, &cloud);
  if (!success) {
    cout << "Error: scene " << name << " not found." << endl;
    return;
  }
  *scene_name_ = name;
  *landmark_scene_ = cloud;
  viz_->set_scene(cloud);
}

string SetLandmarkSceneCommand::name() const { return "use scene"; }
string SetLandmarkSceneCommand::description() const {
  return "<name> - Use a scene for this landmark.";
}

EditBoxCommand::EditBoxCommand(rapid::perception::Box3DRoiServer* box_server,
			       rapid_msgs::Roi3D* roi)
    : box_server_(box_server), roi_(roi) {}
void EditBoxCommand::Execute(const vector<string>& args) {
  box_server_->set_base_frame("base_link");
  if (roi_->dimensions.x != 0 && roi_->dimensions.y != 0 &&
      roi_->dimensions.z != 0) {
    box_server_->Start(roi_->transform.translation.x,
		       roi_->transform.translation.y,
		       roi_->transform.translation.z, roi_->dimensions.x,
		       roi_->dimensions.y, roi_->dimensions.z);
  } else {
    box_server_->Start();
  }
  cout << "Adjust the ROI marker in rviz." << endl;
  cout << "Press enter to save the box: ";
  string input;
  std::getline(std::cin, input);
  *roi_ = box_server_->roi();
  box_server_->Stop();
}
string EditBoxCommand::name() const { return "edit box"; }
string EditBoxCommand::description() const {
  return "- Edit this landmark's box";
}

SaveLandmarkCommand::SaveLandmarkCommand(NameDb* info_db, NameDb* cloud_db,
					 PointCloud2::Ptr landmark_scene,
					 const string& landmark_name,
					 string* scene_name,
					 rapid_msgs::Roi3D* roi,
					 const string& type)
    : info_db_(info_db),
      cloud_db_(cloud_db),
      landmark_scene_(landmark_scene),
      landmark_name_(landmark_name),
      scene_name_(scene_name),
      roi_(roi),
      type_(type) {}

void SaveLandmarkCommand::Execute(const vector<string>& args) {
  // Insert the landmark info.
  rapid_msgs::LandmarkInfo info;
  info.name = landmark_name_;
  info.scene_name = *scene_name_;
  info.roi = *roi_;
  if (type_ == EditLandmarkCommand::kCreate) {
    info_db_->Insert(landmark_name_, info);
  } else {
    bool success = info_db_->Update(landmark_name_, info);
    if (!success) {
      cout << "Error: could not update landmark with name: " << landmark_name_
	   << endl;
      return;
    }
  }

  // Crop the landmark out of the scene.
  pcl::CropBox<PointXYZRGB> crop;
  PointCloud<PointXYZRGB>::Ptr pcl_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(*landmark_scene_, *pcl_cloud);
  ROS_INFO("Scene %s has %ld points", landmark_name_.c_str(),
	   pcl_cloud->size());
  crop.setInputCloud(pcl_cloud);
  Eigen::Vector4f min_pt(roi_->transform.translation.x - roi_->dimensions.x / 2,
			 roi_->transform.translation.y - roi_->dimensions.y / 2,
			 roi_->transform.translation.z - roi_->dimensions.z / 2,
			 0);
  crop.setMin(min_pt);
  Eigen::Vector4f max_pt(roi_->transform.translation.x + roi_->dimensions.x / 2,
			 roi_->transform.translation.y + roi_->dimensions.y / 2,
			 roi_->transform.translation.z + roi_->dimensions.z / 2,
			 0);
  crop.setMax(max_pt);
  PointCloud<PointXYZRGB>::Ptr output(new PointCloud<PointXYZRGB>);
  crop.filter(*output);
  ROS_INFO("Captured cloud with %ld points", output->size());
  PointCloud2 cloud_out;
  pcl::toROSMsg(*output, cloud_out);

  // Save the landmark.
  if (type_ == EditLandmarkCommand::kCreate) {
    cloud_db_->Insert(landmark_name_, cloud_out);
  } else {
    bool success = cloud_db_->Update(landmark_name_, cloud_out);
    if (!success) {
      cout << "Error: could not update landmark cloud with name: "
	   << landmark_name_ << endl;
      return;
    }
  }
}

string SaveLandmarkCommand::name() const { return "save"; }
string SaveLandmarkCommand::description() const {
  return "- Save this landmark.";
}

EditLandmarkCommand::EditLandmarkCommand(
    NameDb* landmark_info_db, NameDb* landmark_cloud_db, NameDb* scene_info_db,
    NameDb* scene_cloud_db, rapid::perception::Box3DRoiServer* box_server,
    SceneViz* scene_viz, const string& type)
    : landmark_info_db_(landmark_info_db),
      landmark_cloud_db_(landmark_cloud_db),
      scene_info_db_(scene_info_db),
      scene_cloud_db_(scene_cloud_db),
      box_server_(box_server),
      scene_viz_(scene_viz),
      type_(type) {}

void EditLandmarkCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: specify a name for the landmark" << endl;
    return;
  }
  string name(boost::algorithm::join(args, " "));

  // Shared state for this sub-CLI
  PointCloud2::Ptr landmark_scene(new PointCloud2);
  string scene_name("");
  rapid_msgs::Roi3D roi;

  if (type_ == kEdit) {
    rapid_msgs::LandmarkInfo landmark_info;
    bool success = landmark_info_db_->Get(name, &landmark_info);
    if (!success) {
      cout << "Error: landmark " << name << " not found." << endl;
      return;
    }
    roi = landmark_info.roi;

    PointCloud2 db_cloud;
    scene_name = landmark_info.scene_name;
    success = scene_cloud_db_->Get(scene_name, &db_cloud);
    if (!success) {
      cout << "Warning: scene \"" << scene_name << "\" for landmark " << name
	   << " not found. Please add a scene before saving the landmark."
	   << endl;
    } else {
      *landmark_scene = db_cloud;
    }
  }

  // Build sub-CLI
  ListCommand list_scenes(scene_info_db_, ListCommand::kScenes, "scenes",
			  "- List scenes");
  SetLandmarkSceneCommand use_scene(scene_cloud_db_, &scene_name,
				    landmark_scene, scene_viz_);
  EditBoxCommand edit_box(box_server_, &roi);
  SaveLandmarkCommand save(landmark_info_db_, landmark_cloud_db_,
			   landmark_scene, name, &scene_name, &roi, type_);
  rapid::utils::ExitCommand exit;

  string title("Creating landmark " + name);
  if (type_ == kEdit) {
    title = "Editing landmark " + name;
  }
  rapid::utils::CommandLine landmark_cli("Creating landmark " + name);
  landmark_cli.AddCommand(&list_scenes);
  landmark_cli.AddCommand(&use_scene);
  landmark_cli.AddCommand(&edit_box);
  landmark_cli.AddCommand(&save);
  landmark_cli.AddCommand(&exit);

  while (landmark_cli.Next()) {
  }
}

string EditLandmarkCommand::name() const { return type_; }
string EditLandmarkCommand::description() const {
  if (type_ == kCreate) {
    return "<name> - Create a landmark";
  } else {
    return "<name> - Edit a landmark";
  }
}

const char EditLandmarkCommand::kCreate[] = "create";
const char EditLandmarkCommand::kEdit[] = "edit";

RecordSceneCommand::RecordSceneCommand(NameDb* info_db, NameDb* cloud_db)
    : info_db_(info_db), cloud_db_(cloud_db), tf_listener_() {}

void RecordSceneCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: provide a name for this scene." << endl;
    return;
  }
  // Read cloud
  PointCloud2::ConstPtr cloud_in =
      ros::topic::waitForMessage<PointCloud2>("cloud_in", ros::Duration(10));

  // Transform to base link.
  PointCloud2 cloud_out;
  bool success = pcl_ros::transformPointCloud("base_link", *cloud_in, cloud_out,
					      tf_listener_);
  if (!success) {
    ROS_ERROR("Error: Failed to transform point cloud.");
    return;
  }

  // Save to DB
  string name = boost::algorithm::join(args, " ");
  rapid_msgs::SceneInfo info;
  info.name = name;
  info_db_->Insert(name, info);
  cloud_db_->Insert(name, cloud_out);
}

string RecordSceneCommand::name() const { return "record scene"; }
string RecordSceneCommand::description() const {
  return "<name> - Save a new scene";
}

DeleteCommand::DeleteCommand(NameDb* info_db, NameDb* cloud_db,
			     const string& type)
    : info_db_(info_db), cloud_db_(cloud_db), type_(type) {}

void DeleteCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: specify the name to delete." << endl;
    return;
  }
  string name = boost::algorithm::join(args, " ");
  bool info_success = false;
  bool cloud_success = false;
  if (type_ == "landmark") {
    info_success = info_db_->Delete<rapid_msgs::LandmarkInfo>(name);
    cloud_success = cloud_db_->Delete<PointCloud2>(name);
  } else {
    info_success = info_db_->Delete<rapid_msgs::SceneInfo>(name);
    cloud_success = cloud_db_->Delete<PointCloud2>(name);
  }
  if (!info_success || !cloud_success) {
    cout << "Invalid name " << name << ", nothing deleted." << endl;
  }
}
string DeleteCommand::name() const { return "delete"; }
string DeleteCommand::description() const {
  return "<name> - Delete a " + type_;
}

SetInputLandmarkCommand::SetInputLandmarkCommand(NameDb* info_db,
						 NameDb* cloud_db,
						 const ros::Publisher& pub,
						 PoseEstimatorInput* input)
    : info_db_(info_db), cloud_db_(cloud_db), pub_(pub), input_(input) {}

void SetInputLandmarkCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: specify the landmark to use." << endl;
    return;
  }
  string name(boost::algorithm::join(args, " "));
  bool success = info_db_->Get(name, &input_->landmark);
  if (!success) {
    cout << "Error: could not find landmark \"" << name << "\"." << endl;
    return;
  }

  success = cloud_db_->Get(name, &input_->landmark_cloud);
  if (!success) {
    cout << "Error: could not find landmark cloud \"" << name << "\"." << endl;
    return;
  }

  rapid::viz::PublishCloud(pub_, input_->landmark_cloud);
}

string SetInputLandmarkCommand::name() const { return "use landmark"; }
string SetInputLandmarkCommand::description() const {
  return "<name> - Use a landmark";
}

SetInputSceneCommand::SetInputSceneCommand(NameDb* info_db, NameDb* cloud_db,
					   const SceneViz& viz,
					   PoseEstimatorInput* input)
    : info_db_(info_db), cloud_db_(cloud_db), viz_(viz), input_(input) {}

void SetInputSceneCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: specify the scene to use." << endl;
    return;
  }
  string name(boost::algorithm::join(args, " "));
  bool success = info_db_->Get(name, &input_->scene);
  if (!success) {
    cout << "Error: could not find scene \"" << name << "\"." << endl;
    return;
  }

  success = cloud_db_->Get(name, &input_->scene_cloud);
  if (!success) {
    cout << "Error: could not find scene cloud \"" << name << "\"." << endl;
    return;
  }

  viz_.set_scene(input_->scene_cloud);
}

string SetInputSceneCommand::name() const { return "use scene"; }
string SetInputSceneCommand::description() const {
  return "<name> - Use a scene";
}

RunCommand::RunCommand(Estimators* estimators, PoseEstimatorInput* input,
		       const ros::Publisher& output_pub)
    : estimators_(estimators),
      input_(input),
      output_pub_(output_pub),
      matches_() {}

void RunCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: specify the algorithm to use." << endl;
    return;
  }
  const string& algorithm = args[0];
  matches_.clear();

  // TODO(jstn): May need to filter NaNs from the scene
  // if (type_ == "scene") {
  //  // Filter NaNs
  //  vector<int> mapping;
  //  pcl_cloud_filtered->is_dense = false;  // Force check for NaNs
  //  pcl::removeNaNFromPointCloud(*pcl_cloud_filtered, *pcl_cloud_filtered,
  //                               mapping);
  //  ROS_INFO("Filtered NaNs, there are now %ld points",
  //           pcl_cloud_filtered->size());
  //}
  PointCloud<PointXYZRGB>::Ptr landmark_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(input_->landmark_cloud, *landmark_cloud);
  ROS_INFO("Loaded landmark with %ld points", landmark_cloud->size());
  PointCloud<PointXYZRGB>::Ptr scene_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(input_->scene_cloud, *scene_cloud);
  PointCloud<PointXYZRGB>::Ptr scene_cropped(new PointCloud<PointXYZRGB>);
  CropScene(scene_cloud, scene_cropped);

  double leaf_size = 0.01;
  ros::param::param<double>("leaf_size", leaf_size, 0.01);

  if (algorithm == "custom") {
    UpdateCustomParams();
    // Downsample landmark and scene
    PointCloud<PointXYZRGB>::Ptr landmark_downsampled(
	new PointCloud<PointXYZRGB>);
    Downsample(leaf_size, landmark_cloud, landmark_downsampled);
    ROS_INFO("Downsampled landmark to %ld points",
	     landmark_downsampled->size());
    PointCloud<PointXYZRGB>::Ptr scene_downsampled(new PointCloud<PointXYZRGB>);
    Downsample(leaf_size, scene_cropped, scene_downsampled);
    ROS_INFO("Downsampled scene to %ld points", scene_downsampled->size());

    pcl::ScopeTime timer(("Running algorithm: " + algorithm).c_str());
    estimators_->custom->set_scene(scene_downsampled);
    estimators_->custom->set_roi(input_->landmark.roi);
    estimators_->custom->set_object(landmark_downsampled);
    estimators_->custom->Find(&matches_);
  } else if (algorithm == "ransac") {
    UpdateRansacParams();
    if (estimators_->ransac->voxel_size() != leaf_size) {
      estimators_->ransac->set_voxel_size(leaf_size);
    }
    pcl::ScopeTime timer(("Running algorithm: " + algorithm).c_str());
    estimators_->ransac->set_object(landmark_cloud);
    estimators_->ransac->set_scene(scene_cropped);
    estimators_->ransac->Find(&matches_);
  } else if (algorithm == "grouping") {
    UpdateGroupingParams();
    pcl::ScopeTime timer(("Running algorithm: " + algorithm).c_str());
    estimators_->grouping->set_object(landmark_cloud);
    estimators_->grouping->set_scene(scene_cropped);
    estimators_->grouping->Find(&matches_);
  } else {
    ROS_ERROR("Unknown algorithm: %s", algorithm.c_str());
    return;
  }
  VisualizeMatches(output_pub_, matches_);
}

string RunCommand::name() const { return "run"; }
string RunCommand::description() const {
  return "<custom, ransac, grouping> - Run object search";
}

void RunCommand::matches(vector<PoseEstimationMatch>* matches) {
  *matches = matches_;
}

void RunCommand::UpdateCustomParams() {
  double sample_ratio;
  int max_samples;
  int num_candidates;
  double fitness_threshold;
  double sigma_threshold;
  double nms_radius;
  int min_results;

  ros::param::param<double>("sample_ratio", sample_ratio, 0.01);
  ros::param::param<int>("max_samples", max_samples, 1000);
  ros::param::param<int>("num_candidates", num_candidates, 100);
  ros::param::param<double>("fitness_threshold", fitness_threshold, 0.0055);
  ros::param::param<double>("sigma_threshold", sigma_threshold, 2);
  ros::param::param<double>("nms_radius", nms_radius, 0.03);
  ros::param::param<int>("min_results", min_results, 0);
  ROS_INFO(
      "Parameters:\n"
      "sample_ratio: %f\n"
      "max_samples: %d\n"
      "num_candidates: %d\n"
      "fitness_threshold: %f\n"
      "sigma_threshold: %f\n"
      "nms_radius: %f\n"
      "min_results: %d\n",
      sample_ratio, max_samples, num_candidates, fitness_threshold,
      sigma_threshold, nms_radius, min_results);

  if (estimators_->custom->heat_mapper()->name() == "random") {
    rapid::perception::RandomHeatMapper* mapper =
	static_cast<rapid::perception::RandomHeatMapper*>(
	    estimators_->custom->heat_mapper());
    mapper->set_sample_ratio(sample_ratio);
    mapper->set_max_samples(max_samples);
  }
  estimators_->custom->set_num_candidates(num_candidates);
  estimators_->custom->set_fitness_threshold(fitness_threshold);
  estimators_->custom->set_sigma_threshold(sigma_threshold);
  estimators_->custom->set_nms_radius(nms_radius);
  estimators_->custom->set_min_results(min_results);
}

void RunCommand::UpdateRansacParams() {
  double pair_width;
  double scene_radius;
  double object_radius;
  double ransac_threshold;
  ros::param::param<double>("pair_width", pair_width, 0.025);
  ros::param::param<double>("scene_radius", scene_radius, 0.025);
  ros::param::param<double>("object_radius", object_radius, 0.01);
  ros::param::param<double>("ransac_threshold", ransac_threshold, 0.3);
  ROS_INFO(
      "Parameters:\n"
      "pair_width: %f\n"
      "scene_radius: %f\n"
      "object_radius: %f\n"
      "ransac_threshold: %f\n",
      pair_width, scene_radius, object_radius, ransac_threshold);

  if (estimators_->ransac->pair_width() != pair_width) {
    estimators_->ransac->set_pair_width(pair_width);
  }
  estimators_->ransac->set_scene_normal_radius(scene_radius);
  estimators_->ransac->set_object_normal_radius(object_radius);
  estimators_->ransac->set_threshold(ransac_threshold);
}

void RunCommand::UpdateGroupingParams() {
  int normal_k;
  double shot_radius;
  double object_vox;
  double scene_vox;
  double corr_match_threshold;
  bool use_hough;
  double rf_radius;
  double cg_size;
  double cg_threshold;

  ros::param::param<int>("normal_k", normal_k,
			 GroupingPoseEstimator::kDefaultNormalK);
  ros::param::param<double>("shot_radius", shot_radius,
			    GroupingPoseEstimator::kDefaultShotRadius);
  ros::param::param<double>("object_vox", object_vox,
			    GroupingPoseEstimator::kDefaultObjectVox);
  ros::param::param<double>("scene_vox", scene_vox,
			    GroupingPoseEstimator::kDefaultSceneVox);
  ros::param::param<double>("corr_match_threshold", corr_match_threshold,
			    GroupingPoseEstimator::kDefaultCorrMatchThreshold);
  ros::param::param<bool>("use_hough", use_hough,
			  GroupingPoseEstimator::kDefaultUseHough);
  ros::param::param<double>("rf_radius", rf_radius,
			    GroupingPoseEstimator::kDefaultRfRadius);
  ros::param::param<double>("cg_size", cg_size,
			    GroupingPoseEstimator::kDefaultCgSize);
  ros::param::param<double>("cg_threshold", cg_threshold,
			    GroupingPoseEstimator::kDefaultCgThreshold);

  ROS_INFO(
      "Parameters:\n"
      "normal_k: %d\n"
      "shot_radius: %f\n"
      "object_vox: %f\n"
      "scene_vox: %f\n"
      "corr_match_threshold: %f\n"
      "use_hough: %d\n"
      "rf_radius: %f\n"
      "cg_size: %f\n"
      "cg_threshold: %f\n",
      normal_k, shot_radius, object_vox, scene_vox, corr_match_threshold,
      use_hough, rf_radius, cg_size, cg_threshold);

  estimators_->grouping->normal_k_ = normal_k;
  estimators_->grouping->shot_radius_ = shot_radius;
  estimators_->grouping->object_vox_ = object_vox;
  estimators_->grouping->scene_vox_ = scene_vox;
  estimators_->grouping->corr_match_threshold_ = corr_match_threshold;
  estimators_->grouping->use_hough_ = use_hough;
  estimators_->grouping->rf_radius_ = rf_radius;
  estimators_->grouping->cg_size_ = cg_size;
  estimators_->grouping->cg_threshold_ = cg_threshold;
}

void RunCommand::CropScene(PointCloud<PointXYZRGB>::Ptr scene,
			   PointCloud<PointXYZRGB>::Ptr cropped) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param<double>("min_x", min_x, 0.2);
  ros::param::param<double>("min_y", min_y, -1);
  ros::param::param<double>("min_z", min_z, 0.3);
  ros::param::param<double>("max_x", max_x, 1.2);
  ros::param::param<double>("max_y", max_y, 1);
  ros::param::param<double>("max_z", max_z, 1.7);
  ROS_INFO(
      "Cropping:\n"
      "  min_x: %f\n"
      "  min_y: %f\n"
      "  min_z: %f\n"
      "  max_x: %f\n"
      "  max_y: %f\n"
      "  max_z: %f\n",
      min_x, min_y, min_z, max_x, max_y, max_z);

  pcl::CropBox<PointXYZRGB> crop;
  crop.setInputCloud(scene);
  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMin(min);
  crop.setMax(max);
  crop.filter(*cropped);
  ROS_INFO("Cropped to %ld points", cropped->size());
}

void RunCommand::Downsample(const double leaf_size,
			    PointCloud<PointXYZRGB>::Ptr cloud_in,
			    PointCloud<PointXYZRGB>::Ptr cloud_out) {
  pcl::VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud_in);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*cloud_out);
}

SetDebugCommand::SetDebugCommand(Estimators* estimators)
    : estimators_(estimators) {}

void SetDebugCommand::Execute(const vector<string>& args) {
  if (args.size() == 0) {
    cout << "Error: specify on or off." << endl;
    return;
  }

  if (args[0] == "on") {
    estimators_->custom->set_debug(true);
  } else {
    estimators_->custom->set_debug(false);
  }
}

string SetDebugCommand::name() const { return "debug"; }
string SetDebugCommand::description() const {
  return "<on/off> - Turn debugging on or off";
}
}  // namespace object_search
