#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_H_

#include <vector>

#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_perception/pose_estimation_heat_mapper.h"

namespace rapid {
namespace perception {
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
  void set_object(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object,
                  const rapid_msgs::Roi3D& roi);

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
  // Pick the top candidates from the heat map.
  void ComputeTopCandidates(Eigen::VectorXd& importances,
                            pcl::PointIndicesPtr heatmap_indices,
                            pcl::PointIndicesPtr candidate_indices);
  // Initialize and run ICP at each of the candidate points.
  void RunIcpCandidates(pcl::PointIndices::Ptr candidate_indices,
                        std::vector<PoseEstimationMatch>* output_objects);
  // Do non-max suppression on ICP outputs.
  void NonMaxSuppression(std::vector<PoseEstimationMatch>& output_objects,
                         std::vector<bool>* keep);
  // Computes an alternative ICP fitness score between the scene and an object
  // with a bounding box around it. This metric measures the distance between
  // the
  // scene points inside the bounding box and the nearest point on the object.
  // This allows us to assert that there should be free space in the given
  // bounding box.
  //
  // We assume that the scene, object, and ROI are all given in the same
  // reference
  // frame.
  //
  // We do not take into account the difference between free space that is
  // visibly
  // free or free space with unknown occupancy (in practice it may not make a
  // difference?).
  //
  // Args:
  //  scene: The scene the object is in.
  //  object: The object to compute the score of.
  //  roi: The region of interest to get scene points from.
  double ComputeIcpFitness(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scene,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object,
                           const rapid_msgs::Roi3D& roi);

  PoseEstimationHeatMapper* heat_mapper_;

  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
  Eigen::Vector3d object_center_;  // Approx center point of object.
  rapid_msgs::Roi3D object_roi_;

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

// Set the color of a point cloud for visualization.
void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b);
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_H_
