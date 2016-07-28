#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_H_

#include <vector>

#include "opencv2/core/core.hpp"
#include "pcl/features/fpfh_omp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"
#include "Eigen/Core"

#include "rapid_perception/image_recognition.h"

namespace rapid {
namespace perception {
// Base class for a heat mapper for pose estimation.
// All heat mappers get a debug flag and a point cloud publisher.
// The scene, object, parameters, etc. are passed into subclasses.
class PoseEstimationHeatMapper {
 public:
  PoseEstimationHeatMapper();
  virtual ~PoseEstimationHeatMapper() {}

  // Generates a heatmap, represented as a list of indices into the cloud
  // representing the scene and a parallel list of "importance" scores.
  virtual void Compute(pcl::PointIndicesPtr indices,
                       Eigen::VectorXd* importances) = 0;

  virtual void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene) = 0;
  virtual void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object) = 0;
  void set_debug(bool val);
  void set_heatmap_publisher(const ros::Publisher& pub);

 protected:
  bool debug_;
  ros::Publisher heatmap_pub_;
};

// Heat mapper that uses CNN features on 2D images of the scene and object.
class CnnHeatMapper : public PoseEstimationHeatMapper {
 public:
  CnnHeatMapper();

  void Compute(pcl::PointIndicesPtr indices, Eigen::VectorXd* importances);

  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_scene_camera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);
  void set_object_camera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);

  void set_image_recognizer(const ImageRecognizer& val);

  void set_sample_ratio(double val);
  void set_max_samples(int val);
  void set_max_sample_radius(double val);
  void set_max_neighbors(int val);
  void set_cnn_layer(std::string val);

  void set_landmark_image_publisher(const ros::Publisher& pub);
  void set_scene_image_publisher(const ros::Publisher& pub);

 private:
  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_camera_;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr scene_tree_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_camera_;
  double object_radius_est_;  // Object radius estimate.
  cv::Mat object_image_;

  ImageRecognizer image_recognizer_;

  // Parameters
  // The fraction of points in the scene to randomly sample when searching for
  // pose.
  double sample_ratio_;
  // The maximum number of samples to search, regardless of the sample ratio.
  int max_samples_;
  // The maximum radius around each sample to look for neighbors. The system
  // generally looks in a radius roughly the size of the object.
  double max_sample_radius_;
  // The maximum number of neighbors, regardless of the sample radius.
  int max_neighbors_;
  // CNN feature to use
  std::string cnn_layer_;

  // Visualization publishers
  ros::Publisher landmark_image_pub_;
  ros::Publisher scene_image_pub_;
};

// Heat mapper that uses FPFH features on the point cloud.
class FpfhHeatMapper : public PoseEstimationHeatMapper {
 public:
  FpfhHeatMapper();

  void Compute(pcl::PointIndicesPtr indices, Eigen::VectorXd* importances);

  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);

  void set_sample_ratio(double val);
  void set_max_samples(int val);
  void set_max_sample_radius(double val);
  void set_max_neighbors(int val);
  void set_feature_threshold(double val);

 private:
  void ComputeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
  void ComputeFeatures(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in,
                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr out);
  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_;
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr scene_tree_;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object_;
  double object_radius_est_;  // Object radius estimate.
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features_;
  pcl::search::KdTree<pcl::FPFHSignature33>::Ptr object_features_tree_;

  // Parameters
  double normal_radius_;   // Radius to use when computing normals
  double feature_radius_;  // Radius to use when computing features
  // The fraction of points in the scene to randomly sample when searching for
  // pose.
  double sample_ratio_;
  // The maximum number of samples to search, regardless of the sample ratio.
  int max_samples_;
  // The maximum radius around each sample to look for neighbors. The system
  // generally looks in a radius roughly the size of the object.
  double max_sample_radius_;
  // The maximum number of neighbors, regardless of the sample radius.
  int max_neighbors_;
  // The threshold in feature distance, below which a feature is considered to
  // be a "close match." For FPFH features, feature distances tend to range from
  // 500 - 8000 (just a rough estimate), and a possibly good threshold is 1500.
  double feature_threshold_;
};

class PoseEstimationMatch {
 public:
  explicit PoseEstimationMatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               double fitness);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud();
  pcl::PointXYZ center() const;
  double fitness() const;
  void set_fitness(double fitness);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointXYZ center_;
  double fitness_;
};

class PoseEstimator {
 public:
  PoseEstimator();
  // Compute scene points to initialize ICP at.
  void set_heat_mapper(PoseEstimationHeatMapper* heat_mapper);

  // Setters for source and target
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);

  PoseEstimationHeatMapper* heat_mapper();

  // Setters for parameters
  void set_num_candidates(int val);
  void set_fitness_threshold(double val);
  void set_nms_radius(double val);

  void set_debug(bool val);

  // Setters for visualization publishers
  void set_candidates_publisher(const ros::Publisher& pub);
  void set_alignment_publisher(const ros::Publisher& pub);
  void set_output_publisher(const ros::Publisher& pub);

  // Method for doing the alignment
  bool Find();

 private:
  // Sample the heatmap by importance to generate candidate points.
  void ComputeCandidates(Eigen::VectorXd& importances,
                         pcl::PointIndicesPtr heatmap_indices,
                         pcl::PointIndicesPtr candidate_indices);
  // Initialize and run ICP at each of the candidate points.
  void RunIcpCandidates(pcl::PointIndices::Ptr candidate_indices,
                        std::vector<PoseEstimationMatch>* output_objects);
  // Do non-max suppression on ICP outputs.
  void NonMaxSuppression(std::vector<PoseEstimationMatch>& output_objects,
                         std::vector<bool>* keep);

  PoseEstimationHeatMapper* heat_mapper_;

  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
  Eigen::Vector3d object_center_;  // Approx center point of object.

  // Parameters
  // Number of candidate samples to consider for ICP.
  int num_candidates_;
  // The ICP fitness threshold below which we consider the alignment a positive
  // match
  double fitness_threshold_;
  // Radius to look for other matches when doing non-max suppression
  double nms_radius_;

  bool debug_;

  // Visualization publishers
  ros::Publisher candidates_pub_;
  ros::Publisher alignment_pub_;
  ros::Publisher output_pub_;
};

// Convert a feature to a string for debugging purposes.
std::string FpfhFeatureString(const pcl::FPFHSignature33& feature);

// Set the color of a point cloud for visualization.
void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b);
// Publish a point cloud using the given publisher.
void PublishCloud(const ros::Publisher& pub,
                  pcl::PointCloud<pcl::PointXYZRGB>& cloud);
void PublishCloud(const ros::Publisher& pub,
                  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);
void PublishImage(const ros::Publisher& pub, const pcl::PCLHeader& cloud_header,
                  const cv::Mat& mat);

}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_H_
