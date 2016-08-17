#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_CNN_HEAT_MAPPER_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_CNN_HEAT_MAPPER_H_

#include "Eigen/Core"
#include "opencv2/core/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"

#include "rapid_perception/image_recognition.h"
#include "rapid_perception/pose_estimation_heat_mapper.h"

namespace rapid {
namespace perception {
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
  // CNN feature to use
  std::string cnn_layer_;

  // Visualization publishers
  ros::Publisher landmark_image_pub_;
  ros::Publisher scene_image_pub_;
};

}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_CNN_HEAT_MAPPER_H_
