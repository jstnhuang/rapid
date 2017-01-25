#ifndef _RAPID_PERCEPTION_GROUPING_POSE_ESTIMATOR_H_
#define _RAPID_PERCEPTION_GROUPING_POSE_ESTIMATOR_H_

#include "rapid_perception/pose_estimation_interface.h"

#include <vector>

#include "Eigen/Dense"
#include "pcl/correspondence.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/shot_omp.h"

#include "ros/ros.h"

namespace rapid {
namespace perception {
// PoseEstimator based on correspondence grouping.
class GroupingPoseEstimator : public PoseEstimationInterface {
 public:
  GroupingPoseEstimator();
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object);
  void Find(std::vector<PoseEstimationMatch>* matches);

  void set_correspondence_publisher(const ros::Publisher& pub);
  void set_scene_keypoints_publisher(const ros::Publisher& pub);
  void set_object_keypoints_publisher(const ros::Publisher& pub);

  // Default parameter values
  const static int kDefaultNormalK;
  const static double kDefaultShotRadius;
  const static double kDefaultObjectVox;
  const static double kDefaultSceneVox;
  const static double kDefaultCorrMatchThreshold;
  const static bool kDefaultUseHough;
  const static double kDefaultRfRadius;
  const static double kDefaultCgSize;
  const static double kDefaultCgThreshold;

  // Parameters
  int normal_k_;
  double shot_radius_;
  double object_vox_;
  double scene_vox_;
  double corr_match_threshold_;
  bool use_hough_;
  double rf_radius_;
  double cg_size_;
  double cg_threshold_;

 private:
  void ComputeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr normals);
  void ComputeKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                        const double sample_radius,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints);
  void ComputeDescriptors(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints,
      const pcl::PointCloud<pcl::Normal>::Ptr& normals,
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surface,
      pcl::PointCloud<pcl::SHOT352>::Ptr descriptors);
  void FindCorrespondences(pcl::CorrespondencesPtr model_scene_corrs);
  void ClusterCorrespondences(
      const pcl::CorrespondencesPtr& model_scene_corrs,
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >*
          rototranslations,
      std::vector<pcl::Correspondences>* clustered_corrs);

  // Data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints_;
  pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
  pcl::PointCloud<pcl::Normal>::Ptr object_normals_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_keypoints_;
  pcl::PointCloud<pcl::SHOT352>::Ptr object_descriptors_;

  ros::Publisher correspondence_pub_;
  ros::Publisher scene_keypoints_pub_;
  ros::Publisher object_keypoints_pub_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_GROUPING_POSE_ESTIMATOR_H_
