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
class PoseEstimationMatch {
 public:
  explicit PoseEstimationMatch(
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, double fitness);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud();
  pcl::PointXYZ center() const;
  double fitness() const;
  void set_fitness(double fitness);

 private:
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_;
  pcl::PointXYZ center_;
  double fitness_;
};

class PoseEstimator {
 public:
  PoseEstimator();
  // Setters for source and target
  void set_scene(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr scene);
  void set_scene_camera(
      pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr object);
  void set_object_camera(
      pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr object);
  void set_scene_features(
      pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr scene_features);
  void set_object_features(
      pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr object_features);

  // Setters for parameters
  void set_sample_ratio(double val);
  void set_max_samples(int val);
  void set_max_sample_radius(double val);
  void set_max_neighbors(int val);
  void set_feature_threshold(double val);
  void set_num_candidates(int val);
  void set_fitness_threshold(double val);
  void set_nms_radius(double val);

  void set_debug(bool val);

  // Setters for visualization publishers
  void set_heatmap_publisher(const ros::Publisher& pub);
  void set_initial_publisher(const ros::Publisher& pub);
  void set_candidates_publisher(const ros::Publisher& pub);
  void set_alignment_publisher(const ros::Publisher& pub);
  void set_output_publisher(const ros::Publisher& pub);
  void set_landmark_image_publisher(const ros::Publisher& pub);
  void set_scene_image_publisher(const ros::Publisher& pub);

  void set_image_recognizer(const ImageRecognizer& val);

  // Method for doing the alignment
  bool Find();

 private:
  // Compute scene points to initialize ICP at.
  void ComputeHeatmap(pcl::PointIndicesPtr heatmap_indices,
                      Eigen::VectorXd* importances);
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

  // Set the color of a point cloud for visualization.
  void Colorize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, double r,
                double g, double b);

  // Publish a point cloud using the given publisher.
  void PublishCloud(const ros::Publisher& pub,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);
  void PublishImage(const ros::Publisher& pub,
                    const pcl::PCLHeader& cloud_header, const cv::Mat& mat);

  // Convert a feature to a string for debugging purposes.
  std::string FeatureString(const pcl::FPFHSignature33& feature);

  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_rgb_camera_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_rgb_;
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr scene_tree_;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_rgb_camera_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_rgb_;
  double object_radius_est_;       // Object radius estimate.
  Eigen::Vector3d object_center_;  // Approx center point of object.
  cv::Mat object_cnn_features_;
  double object_cnn_norm_;

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features_;

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features_;
  pcl::search::KdTree<pcl::FPFHSignature33>::Ptr object_features_tree_;

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
  // The threshold in feature distance, below which a feature is considered to
  // be a "close match." For FPFH features, feature distances tend to range from
  // 500 - 8000 (just a rough estimate), and a possibly good threshold is 1500.
  double feature_threshold_;
  // Number of candidate samples to consider for ICP.
  int num_candidates_;
  // The ICP fitness threshold below which we consider the alignment a positive
  // match
  double fitness_threshold_;
  // Radius to look for other matches when doing non-max suppression
  double nms_radius_;

  bool debug_;

  // Visualization publishers
  ros::Publisher heatmap_pub_;
  ros::Publisher initial_pub_;
  ros::Publisher candidates_pub_;
  ros::Publisher alignment_pub_;
  ros::Publisher output_pub_;
  ros::Publisher landmark_image_pub_;
  ros::Publisher scene_image_pub_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_H_
