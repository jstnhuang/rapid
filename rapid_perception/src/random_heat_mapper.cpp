#include "rapid_perception/random_heat_mapper.h"

#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
using std::vector;

namespace rapid {
namespace perception {
RandomHeatMapper::RandomHeatMapper()
    : scene_(), sample_ratio_(0.05), max_samples_(1000) {}

void RandomHeatMapper::Compute(pcl::PointIndicesPtr indices,
                               Eigen::VectorXd* importances) {
  indices->indices.clear();

  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  num_samples = std::min(num_samples, max_samples_);

  pcl::RandomSample<PointC> random;
  random.setSeed(0);
  random.setSample(num_samples);
  random.setInputCloud(scene_);
  random.filter(indices->indices);

  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  importances->resize(num_samples);
  importances->fill(1);

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

void RandomHeatMapper::set_scene(PointCloudC::Ptr scene) { scene_ = scene; }

void RandomHeatMapper::set_object(PointCloudC::Ptr object) {
  // This function intentionally left empty.
}

void RandomHeatMapper::set_sample_ratio(double val) { sample_ratio_ = val; }

void RandomHeatMapper::set_max_samples(int val) { max_samples_ = val; }
}  // namespace perception
}  // namespace rapid
