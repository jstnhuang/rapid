#include "rapid_perception/random_heat_mapper.h"

#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
using std::vector;

namespace rapid {
namespace perception {
RandomHeatMapper::RandomHeatMapper()
    : scene_(),
      sample_ratio_(0.05),
      max_samples_(1000),
      scale_x_(0.02),
      scale_y_(0.02),
      scale_z_(0.02) {}

void RandomHeatMapper::Compute(PointCloudC::Ptr heatmap,
                               Eigen::VectorXd* importances) {
  double scale;
  ros::param::param("random_scale", scale, 2.0);
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(scene_);
  vox.setLeafSize(scale_x_ / scale, scale_y_ / scale, scale_z_ / scale);
  vox.filter(*heatmap);

  ROS_INFO("Randomly sampled %ld points", heatmap->size());

  importances->resize(heatmap->size());
  importances->fill(1);

  viz::PublishCloud(heatmap_pub_, *heatmap);
}

void RandomHeatMapper::set_scene(PointCloudC::Ptr scene) { scene_ = scene; }

void RandomHeatMapper::set_object(PointCloudC::Ptr object) {
  // This function intentionally left empty.
}

void RandomHeatMapper::set_sample_ratio(double val) { sample_ratio_ = val; }

void RandomHeatMapper::set_max_samples(int val) { max_samples_ = val; }

void RandomHeatMapper::set_landmark_dimensions(double x, double y, double z) {
  scale_x_ = x;
  scale_y_ = y;
  scale_z_ = z;
}
}  // namespace perception
}  // namespace rapid
