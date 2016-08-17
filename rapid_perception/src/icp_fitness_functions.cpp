#include "rapid_perception/icp_fitness_functions.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_viz/publish.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "rapid_viz/markers.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::search::KdTree<PointC> PointCTree;
using std::vector;

namespace rapid {
namespace perception {
double ComputeIcpFitness(
    PointCloudC::Ptr& scene, PointCloudC::Ptr& object,
    const rapid_msgs::Roi3D& roi, bool debug,
    rapid_ros::Publisher<visualization_msgs::Marker>* pub) {
  Eigen::Vector4f min;
  min.x() = -roi.dimensions.x / 2;
  min.y() = -roi.dimensions.y / 2;
  min.z() = -roi.dimensions.z / 2;
  Eigen::Vector4f max;
  max.x() = roi.dimensions.x / 2;
  max.y() = roi.dimensions.y / 2;
  max.z() = roi.dimensions.z / 2;
  Eigen::Vector3f translation;
  translation.x() = roi.transform.translation.x;
  translation.y() = roi.transform.translation.y;
  translation.z() = roi.transform.translation.z;
  Eigen::Quaternionf rotation;
  rotation.w() = roi.transform.rotation.w;
  rotation.x() = roi.transform.rotation.x;
  rotation.y() = roi.transform.rotation.y;
  rotation.z() = roi.transform.rotation.z;
  Eigen::Matrix3f rot_mat = rotation.toRotationMatrix();
  Eigen::Vector3f ea;
  ea.x() = atan2(rot_mat(2, 1), rot_mat(2, 2));
  ea.y() = asin(-rot_mat(2, 0));
  ea.z() = atan2(rot_mat(1, 0), rot_mat(0, 0));

  pcl::CropBox<PointC> crop;
  crop.setInputCloud(scene);
  crop.setMin(min);
  crop.setMax(max);
  crop.setTranslation(translation);
  crop.setRotation(ea);
  pcl::IndicesPtr indices(new vector<int>);
  crop.filter(*indices);

  vector<bool> visited(object->size());
  PointCTree object_tree;
  object_tree.setInputCloud(object);
  vector<int> nn_indices(1);
  vector<float> nn_dists(1);
  double fitness = 0;
  int denominator = 0;
  for (size_t index_i = 0; index_i < indices->size(); ++index_i) {
    int index = indices->at(index_i);
    const PointC& scene_pt = scene->at(index);
    object_tree.nearestKSearch(scene_pt, 1, nn_indices, nn_dists);
    double distance = sqrt(nn_dists[0]);
    if (distance < 0.05) {
      fitness += distance;
      ++denominator;
    }
    visited[nn_indices[0]] = true;
  }

  // Compute distance for unvisited object points
  bool scene_tree_built = false;
  PointCTree scene_tree;
  for (size_t i = 0; i < visited.size(); ++i) {
    if (!visited[i]) {
      if (!scene_tree_built) {
        scene_tree.setInputCloud(scene, indices);
        scene_tree_built = true;
      }
      const PointC& obj_pt = object->at(i);
      scene_tree.nearestKSearch(obj_pt, 1, nn_indices, nn_dists);
      double distance = sqrt(nn_dists[0]);
      if (distance < 0.05) {
        fitness += distance;
        ++denominator;
      }
    }
  }

  fitness /= denominator;

  // if (debug && pub != NULL) {
  //  geometry_msgs::PoseStamped ps;
  //  ps.header.frame_id = object->header.frame_id;
  //  ps.pose.position.x = translation.x();
  //  ps.pose.position.y = translation.y();
  //  ps.pose.position.z = translation.z();
  //  ps.pose.orientation.w = rotation.w();
  //  ps.pose.orientation.x = rotation.x();
  //  ps.pose.orientation.y = rotation.y();
  //  ps.pose.orientation.z = rotation.z();
  //  geometry_msgs::Vector3 scale;
  //  scale.x = roi.dimensions.x;
  //  scale.y = roi.dimensions.y;
  //  scale.z = roi.dimensions.z;
  //  viz::Marker box = viz::Marker::Box(pub, ps, scale);
  //  box.SetColor(1, 0, 0, 0.25);
  //  box.Publish();
  //  std::string input;
  //  std::cout << "Fitness: " << fitness
  //            << ". Press enter to continue: " << std::endl;
  //  std::getline(std::cin, input);
  //}

  return fitness;
}
}  // namespace perception
}  // namespace rapid
