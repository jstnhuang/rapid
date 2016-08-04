#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_H_

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_perception/pose_estimation_heat_mapper.h"
#include "rapid_ros/publisher.h"

namespace rapid {
namespace perception {
class PoseEstimationMatch {
 public:
  // Do not use default constructor except to initialize empty objects.
  PoseEstimationMatch();
  PoseEstimationMatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                      double fitness);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud() const;
  pcl::PointXYZ center() const;
  double fitness() const;
  void set_fitness(double fitness);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointXYZ center_;
  double fitness_;
};

// Finds instances of a given object in a scene. It is designed and tested to
// work with single-view point clouds that are voxelized to a resolution of a
// 0.5cm.
//
// In addition to providing an object, you must also provide a region of
// interest, represented as a box, around the object. Empty space in your box
// matters -- the algorithm will look for that space to be empty when
// considering candidate poses for the object.
//
// Usage:
//  RandomHeatMapper* heat_mapper = new RandomHeatMapper();
//  PoseEstimator estimator(heat_mapper);
//  estimator.set_scene(scene);
//  estimator.set_object(object, object_roi);
//
//  estimator.set_fitness_threshold(0.0045);
//  estimator.set_min_results(1); // Overrides the fitness threshold if set.
//  vector<PoseEstimationMatch> matches;
//  estimator.Find(&matches);
//
// There are two ways to use the algorithm. You can ask it to return all matches
// whose fitness scores are less than the given fitness threshold (by default,
// 0.45cm). Note that the lower the fitness, the better the match.
//
// You can also ask it to return at least N results, even if some or all of
// those results are above the fitness threshold. This is helpful when you
// already know from context that N instances of the object are in the scene.
class PoseEstimator {
 public:
  // Constructs a pose estimator given a heat mapper. We do not take ownership
  // over the pointer.
  PoseEstimator(PoseEstimationHeatMapper* heat_mapper);

  // Set the scene to search for the object in. We recommend voxelizing it for
  // speed at the same resolution as the object. A resolution of 0.5cm works
  // well.
  //
  // We assume it has already been filtered for NaNs. The scene should be in the
  // same frame as the object.
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);

  // Set the object to search for. We recommend voxelizing it for speed at the
  // same resolution as the scene. A resolution of 0.5cm works well.
  //
  // We assume it has already been filtered for NaNs. The object should be in
  // the same frame as the scene.
  //
  // The region of interest (ROI) is a box around the object, which indicates
  // how much empty space around the object you want to enforce. For example, if
  // the object is a horizontal cross-section of a cylinder, and there's no
  // empty space on top or bottom, then many such cross-sections may be found on
  // a single cylinder. However, if the ROI includes empty space above the cross
  // section, then the algorithm will only find the top-most cross section of
  // the cylinder (e.g., the cap of a bottle).
  void set_object(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object,
                  const rapid_msgs::Roi3D& roi);

  PoseEstimationHeatMapper* heat_mapper();

  // Setters for parameters
  // num_candidates is the number of samples to initialize ICP at. More
  // candidates will improve recall but possibly decrease precision, and take
  // longer. The default value is 100.
  void set_num_candidates(int val);

  // sigma_threshold can sometimes help find objects, although we don't
  // recommend using it as it's not very precise.
  void set_sigma_threshold(double val);

  // nms_radius is the radius to look around each match to eliminate worse
  // matches. Often, many matches will be clustered in the same location, so we
  // want to consolidate all of them into the single best match in the cluster.
  // The default value is 0.02.
  void set_nms_radius(double val);

  // If the fitness threshold is set, then we return matches whose fitness
  // values are below the threshold. A higher threshold will lead to higher
  // recall but lower precision. The default value is 0.0045. It can be
  // interpreted as the average error distance between points on the object and
  // points in the scene (within the object's ROI).
  void set_fitness_threshold(double val);

  // Sets the minimum number of matches to return, even if some or all of the
  // matches are above the fitness threshold. This can be helpful if you already
  // know from context that a match exists. Often, the best match is correct
  // even though it's above the fitness threshold. The default value is 0.
  void set_min_results(int val);

  void set_debug(bool val);

  // Setters for visualization publishers
  void set_candidates_publisher(const ros::Publisher& pub);
  void set_alignment_publisher(const ros::Publisher& pub);
  void set_output_publisher(const ros::Publisher& pub);
  void set_marker_publisher(
      rapid_ros::Publisher<visualization_msgs::Marker>* pub);

  // Runs the pose estimation algorithm. It will return all matches whose
  // fitness are below the fitness threshold (lower fitness means a better
  // match). If min_results is set, it will return at least that many matches,
  // even if they are above the fitness threshold.
  void Find(std::vector<PoseEstimationMatch>* matches);

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
                        std::vector<PoseEstimationMatch>* aligned_objects);
  // Do non-max suppression on ICP outputs.
  void NonMaxSuppression(std::vector<PoseEstimationMatch>& aligned_objects,
                         std::vector<int>* deduped_indices);
  // Filter matches based on score and/or statistical tests.
  void FilterMatches(const std::vector<PoseEstimationMatch>& aligned_objects,
                     const std::vector<int>& deduped_indices,
                     std::vector<int>* output_indices);
  void GenerateRotations(std::vector<Eigen::Quaternionf>* rotations);

  PoseEstimationHeatMapper* heat_mapper_;

  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
  Eigen::Vector3f object_center_;  // Approx center point of object.
  rapid_msgs::Roi3D object_roi_;

  // Parameters
  // Number of candidate samples to consider for ICP.
  int num_candidates_;
  // The ICP fitness threshold below which we consider the alignment a positive
  // match
  double fitness_threshold_;
  // The number of standard deviations below the mean fitness with which we'll
  // accept the best match, if none of the matches are below the fitness
  // threshold.
  double sigma_threshold_;
  // Radius to look for other matches when doing non-max suppression
  double nms_radius_;
  // The minimum number of matches to return, regardless of fitness score.
  int min_results_;

  bool debug_;

  // Visualization publishers
  ros::Publisher candidates_pub_;
  ros::Publisher alignment_pub_;
  ros::Publisher output_pub_;
  rapid_ros::Publisher<visualization_msgs::Marker>* marker_pub_;
};

// Set the color of a point cloud for visualization.
void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b);
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_H_
