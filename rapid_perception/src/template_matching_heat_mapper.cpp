#include "rapid_perception/template_matching_heat_mapper.h"

#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/registration/icp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_perception/icp_fitness_functions.h"
#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
using std::vector;

namespace rapid {
namespace perception {
TemplateMatchingHeatMapper::TemplateMatchingHeatMapper()
    : scene_(),
      object_(),
      object_roi_(),
      sample_ratio_(0.05),
      max_samples_(1000) {}

void TemplateMatchingHeatMapper::Compute(pcl::PointIndicesPtr indices,
                                         Eigen::VectorXd* importances) {
  indices->indices.clear();

  // Sample randomly
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  num_samples = std::min(num_samples, max_samples_);
  pcl::RandomSample<PointC> random;
  random.setSeed(0);
  random.setInputCloud(scene_);
  random.setSample(num_samples);
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  PointCloudC::Ptr working_object(new PointCloudC);
  PointCloudC::Ptr aligned_object(new PointCloudC);
  importances->resize(indices->indices.size());
  pcl::IterativeClosestPoint<PointC, PointC> icp;
  icp.setMaximumIterations(0);
  icp.setInputTarget(scene_);
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int index = indices->indices[indices_i];
    const PointC& center = scene_->points[index];
    // Transform the object to be centered on the center point
    // So far we just use a translation offset
    // Also assume that the object and scene are in the same frame
    Eigen::Vector3d translation;
    translation.x() = center.x - object_center_.x();
    translation.y() = center.y - object_center_.y();
    translation.z() = center.z - object_center_.z();
    pcl::transformPointCloud(*object_, *working_object, translation,
                             Eigen::Quaterniond::Identity());

    // Run ICP
    icp.setInputSource(working_object);
    icp.align(*aligned_object);

    rapid_msgs::Roi3D roi = object_roi_;
    roi.transform.translation.x += translation.x();
    roi.transform.translation.y += translation.y();
    roi.transform.translation.z += translation.z();
    double fitness = ComputeIcpFitness(scene_, aligned_object, roi);
    (*importances)(indices_i) = fitness;

    // viz::PublishCloud(heatmap_pub_, *aligned_object);
    // if (debug_) {
    //  std::cout << "Score: " << fitness;
    //  std::string input;
    //  std::getline(std::cin, input);
    //}
  }

  double min = importances->minCoeff();
  importances->array() -= min;
  double max = importances->maxCoeff();
  (*importances) /= max;
  importances->array() = 1 - importances->array();

  // Color point cloud for visualization
  PointCloudC::Ptr working_scene(new PointCloudC);
  *working_scene = *scene_;
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int color = static_cast<int>(round(255 * (*importances)(indices_i)));
    int index = indices->indices[indices_i];
    working_scene->points[index].r = color;
    working_scene->points[index].g = color;
    working_scene->points[index].b = color;
  }
  PointCloudC::Ptr viz_cloud(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(working_scene);
  extract.setIndices(indices);
  extract.filter(*viz_cloud);
  viz::PublishCloud(heatmap_pub_, *viz_cloud);
}

void TemplateMatchingHeatMapper::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
}

void TemplateMatchingHeatMapper::set_object(PointCloudC::Ptr object) {
  object_ = object;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  object_center_.x() = (max_pt.x() + min_pt.x()) / 2;
  object_center_.y() = (max_pt.y() + min_pt.y()) / 2;
  object_center_.z() = (max_pt.z() + min_pt.z()) / 2;
}

void TemplateMatchingHeatMapper::set_object_roi(const rapid_msgs::Roi3D& roi) {
  object_roi_ = roi;
}

void TemplateMatchingHeatMapper::set_sample_ratio(double val) {
  sample_ratio_ = val;
}

void TemplateMatchingHeatMapper::set_max_samples(int val) {
  max_samples_ = val;
}
}  // namespace perception
}  // namespace rapid
