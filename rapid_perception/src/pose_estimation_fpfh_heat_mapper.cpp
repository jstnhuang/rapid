#include "rapid_perception/pose_estimation_fpfh_heat_mapper.h"

#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/common/common.h"
#include "pcl/common/time.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"

#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZRGBNormal PointN;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<FPFH> PointCloudF;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointN> PointCloudN;
typedef pcl::search::KdTree<FPFH> FeatureTree;
typedef pcl::search::KdTree<PointN> PointNTree;

using std::string;
using std::vector;

namespace rapid {
namespace perception {
FpfhHeatMapper::FpfhHeatMapper()
    : scene_(new PointCloudN()),
      scene_tree_(new PointNTree),
      scene_features_(new PointCloudF),
      object_(new PointCloudN()),
      object_radius_est_(0),
      object_features_(new PointCloudF),
      object_features_tree_(new FeatureTree),
      normal_radius_(0.03),
      feature_radius_(0.04),
      sample_ratio_(0.01),
      max_samples_(2000),
      max_sample_radius_(0.05),
      max_neighbors_(400),
      feature_threshold_(1500) {}

void FpfhHeatMapper::Compute(pcl::PointIndicesPtr indices,
                             Eigen::VectorXd* importances) {
  // Sample points in the scene.
  pcl::RandomSample<PointN> random;
  random.setInputCloud(scene_);
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  random.setSample(std::min(max_samples_, num_samples));
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  // For each sample, look at the points around it, project the image into 2D,
  // and use CNN features to compare to the landmark.
  double search_radius = std::min(max_sample_radius_, object_radius_est_);
  // For each scene point, caches whether there is a close match on the object.
  // 0 = unknown, 1 = yes, -1 = no
  vector<int8_t> importance_cache(scene_->size(), 0);
  importances->resize(indices->indices.size());
  pcl::PointIndicesPtr k_indices_ptr(new pcl::PointIndices);
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int index = indices->indices[indices_i];
    vector<int> k_indices;
    vector<float> k_distances;
    scene_tree_->radiusSearch(index, search_radius, k_indices, k_distances,
                              max_neighbors_);
    k_indices_ptr->indices = k_indices;

    double avg_close_matches = 0;
    for (size_t k = 0; k < k_indices.size(); ++k) {
      int k_index = k_indices[k];
      if (importance_cache[k_index] == 1) {
        avg_close_matches += 1;
        continue;
      } else if (importance_cache[k_index] == -1) {
        continue;
      }
      const FPFH& feature = scene_features_->at(k_index);
      vector<int> nearest_indices;
      vector<float> nearest_distances;
      object_features_tree_->nearestKSearch(feature, 1, nearest_indices,
                                            nearest_distances);
      if (nearest_distances[0] < feature_threshold_) {
        avg_close_matches += 1;
        importance_cache[k_index] = 1;
      } else {
        importance_cache[k_index] = -1;
      }
    }
    avg_close_matches /= k_indices.size();
    (*importances)(indices_i) = avg_close_matches;
  }

  double max = importances->maxCoeff();
  (*importances) /= max;

  // Color point cloud for visualization
  PointCloudN::Ptr working_scene(new PointCloudN);
  *working_scene = *scene_;
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int color = static_cast<int>(round(255 * (*importances)(indices_i)));
    int index = indices->indices[indices_i];
    working_scene->points[index].r = color;
    working_scene->points[index].g = color;
    working_scene->points[index].b = color;
  }
  PointCloudN::Ptr viz_cloud(new PointCloudN());
  pcl::ExtractIndices<PointN> extract;
  extract.setInputCloud(working_scene);
  extract.setIndices(indices);
  extract.filter(*viz_cloud);
  viz::PublishCloud(heatmap_pub_, *viz_cloud);
}

void FpfhHeatMapper::set_scene(PointCloudC::Ptr scene) {
  scene_.reset(new PointCloudN);
  pcl::copyPointCloud(*scene, *scene_);
  scene_tree_.reset(new PointNTree());
  scene_tree_->setInputCloud(scene_);
  ComputeNormals(scene_);
  ComputeFeatures(scene_, scene_features_);
}

void FpfhHeatMapper::set_object(PointCloudC::Ptr object) {
  object_.reset(new PointCloudN);
  pcl::copyPointCloud(*object, *object_);

  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  double x = max_pt.x() - min_pt.x();
  double y = max_pt.y() - min_pt.y();
  double z = max_pt.z() - min_pt.z();
  object_radius_est_ = (x + y + z) / 3;

  ComputeNormals(object_);
  ComputeFeatures(object_, object_features_);
  object_features_tree_.reset(new FeatureTree());
  object_features_tree_->setInputCloud(object_features_);
}

void FpfhHeatMapper::set_sample_ratio(double val) { sample_ratio_ = val; }
void FpfhHeatMapper::set_max_samples(int val) { max_samples_ = val; }
void FpfhHeatMapper::set_max_sample_radius(double val) {
  max_sample_radius_ = val;
}
void FpfhHeatMapper::set_max_neighbors(int val) { max_neighbors_ = val; }
void FpfhHeatMapper::set_feature_threshold(double val) {
  feature_threshold_ = val;
}

void FpfhHeatMapper::ComputeNormals(PointCloudN::Ptr cloud) {
  double normal_radius;
  ros::param::param<double>("normal_radius", normal_radius, 0.03);
  ROS_INFO("Computing normals with radius: %f", normal_radius);

  pcl::ScopeTime normal_timer("Computing normals");
  pcl::NormalEstimationOMP<PointN, PointN> nest;
  nest.setRadiusSearch(normal_radius);
  nest.setInputCloud(cloud);
  nest.compute(*cloud);
}

void FpfhHeatMapper::ComputeFeatures(PointCloudN::Ptr in,
                                     PointCloudF::Ptr out) {
  double feature_radius;
  ros::param::param<double>("feature_radius", feature_radius, 0.04);
  ROS_INFO("Computing features with radius: %f", feature_radius);

  pcl::ScopeTime fest_time("Computing features");
  pcl::FPFHEstimationOMP<PointN, PointN, FPFH> fest;
  fest.setRadiusSearch(feature_radius);
  fest.setInputCloud(in);
  fest.setInputNormals(in);
  fest.compute(*out);
}

string FpfhFeatureString(const FPFH& feature) {
  std::stringstream ss;
  for (int i = 0; i < 33; ++i) {
    ss << feature.histogram[i];
    if (i != 32) {
      ss << ", ";
    }
  }
  return ss.str();
}
}  // namespace perception
}  // namespace rapid
