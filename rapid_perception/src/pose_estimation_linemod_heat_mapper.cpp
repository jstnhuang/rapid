#include "rapid_perception/pose_estimation_linemod_heat_mapper.h"

// Heat mapper based on Linemod - only works if point clouds are organized.

#include <algorithm>
#include <math.h>
#include <vector>

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/common/common.h"
#include "pcl/filters/random_sample.h"
#include "pcl/registration/icp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/recognition/line_rgbd.h"

typedef pcl::PointXYZRGB PointC;
// typedef pcl::PointXYZRGBA PointCA;
typedef pcl::PointCloud<PointC> PointCloudC;
// typedef pcl::PointCloud<PointCA> PointCloudCA;
using std::vector;

namespace rapid {
namespace perception {
LinemodHeatMapper::LinemodHeatMapper()
    : scene_(), object_(), sample_ratio_(0.01), max_samples_(100) {}

void LinemodHeatMapper::Compute(pcl::PointIndicesPtr indices,
                                Eigen::VectorXd* importances) {
  // Sample points in the scene.
  pcl::RandomSample<PointC> random;
  random.setInputCloud(scene_);
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  random.setSample(std::min(max_samples_, num_samples));
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  PointCloudC::Ptr working_object(new PointCloudC);
  importances->resize(indices->indices.size());
  pcl::IterativeClosestPoint<PointC, PointC> icp;
  icp.setMaximumIterations(1);
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
  }

  // vector<pcl::LineRGBD<PointC>::Detection> detections;
  // linemod_.detect(detections);

  // for (size_t i = 0; i < detections.size(); ++i) {
  //  const pcl::LineRGBD<PointC>::Detection& detection = detections[i];
  //  ROS_INFO("Bounding box: %f %f %f %f %f %f", detection.bounding_box.x,
  //           detection.bounding_box.y, detection.bounding_box.z,
  //           detection.bounding_box.depth, detection.bounding_box.width,
  //           detection.bounding_box.height);
  //}

  // indices->indices.push_back(0);
  // importances->resize(1);
}

void LinemodHeatMapper::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
  // scene_.reset(new PointCloudCA);
  // pcl::copyPointCloud(*scene, *scene_);
  //  linemod_.setInputCloud(scene_);
  // linemod_.setInputColors(scene_);
}

void LinemodHeatMapper::set_object(PointCloudC::Ptr object) {
  object_ = object;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  object_center_.x() = (max_pt.x() + min_pt.x()) / 2;
  object_center_.y() = (max_pt.y() + min_pt.y()) / 2;
  object_center_.z() = (max_pt.z() + min_pt.z()) / 2;

  // object_.reset(new PointCloudCA);
  // pcl::copyPointCloud(*object, *object_);
  // if (object_->isOrganized()) {
  //  ROS_INFO("obj is organized");
  //} else {
  //  ROS_INFO("obj is not organized");
  //}
  // int object_id = 0;
  // pcl::MaskMap mask_map(object->width, object->height);
  // for (size_t row = 0; row < object->height; ++row) {
  //  for (size_t col = 0; col < object->width; ++col) {
  //    mask_map(col, row) = true;
  //  }
  //}
  // pcl::RegionXY region;
  // region.x = 0;
  // region.y = 0;
  // region.width = object->width;
  // region.height = object->height;

  // pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
  // color_grad_mod.setInputCloud(object_);
  // color_grad_mod.processInputData();

  // pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
  // surface_norm_mod.setInputCloud(object_);
  // surface_norm_mod.processInputData();

  // std::vector<pcl::QuantizableModality*> modalities(2);
  // modalities[0] = &color_grad_mod;
  // modalities[1] = &surface_norm_mod;

  // std::vector<pcl::MaskMap*> masks(2);
  // masks[0] = &mask_map;
  // masks[1] = &mask_map;

  // pcl::LINEMOD linemod;
  // linemod.createAndAddTemplate(modalities, masks, region);
  // linemod_.createAndAddTemplate(rgba, object_id, mask_map, mask_map, region);
}
}  // namespace perception
}  // namespace rapid
