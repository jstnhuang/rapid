#include "rapid_perception/rgbd.hpp"

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/pca.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"

#include <iostream>
#include <limits>
#include <set>
#include <vector>

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;
using pcl::search::KdTree;
using std::cout;
using std::endl;
using std::vector;

namespace rapid {
namespace perception {
void GetPlanarBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  // Project points onto XY plane.
  PointCloud<PointXYZRGB>::Ptr projected(new PointCloud<PointXYZRGB>(cloud));
  for (size_t i = 0; i < projected->points.size(); ++i) {
    PointXYZRGB& point = projected->at(i);
    point.z = 0;
  }

  // Compute PCA.
  pcl::PCA<PointXYZRGB> pca(true);
  pca.setInputCloud(projected);

  // Get eigenvectors.
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  // Because we projected points on the XY plane, we add in the Z vector as the
  // 3rd eigenvector.
  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
  Eigen::Quaternionf q1(eigenvectors);

  // Find min/max x and y, based on the points in eigenspace.
  PointCloud<PointXYZRGB>::Ptr eigen_projected(
      new PointCloud<PointXYZRGB>(cloud));
  pca.project(cloud, *eigen_projected);

  pcl::PointXYZRGB eigen_min;
  pcl::PointXYZRGB eigen_max;
  pcl::getMinMax3D(*eigen_projected, eigen_min, eigen_max);
  double x_length = eigen_max.x - eigen_min.x;
  double y_length = eigen_max.y - eigen_min.y;

  // The points in eigenspace all have z values of 0. Get min/max z from the
  // original point cloud data.
  pcl::PointXYZRGB cloud_min;
  pcl::PointXYZRGB cloud_max;
  pcl::getMinMax3D(cloud, cloud_min, cloud_max);
  double z_length = cloud_max.z - cloud_min.z;

  // Compute midpoint, defined as the midpoint between the minimum and maximum
  // points, in x, y, and z directions. The centroid is an average that depends
  // on the density of points, which doesn't give you the geometric center of
  // the point cloud.
  PointXYZRGB eigen_center;
  eigen_center.x = eigen_min.x + x_length / 2;
  eigen_center.y = eigen_min.y + y_length / 2;
  eigen_center.z = 0;
  PointXYZRGB center;
  pca.reconstruct(eigen_center, center);
  center.z = z_length / 2 + cloud_min.z;

  // Output midpoint.
  midpoint->position.x = center.x;
  midpoint->position.y = center.y;
  midpoint->position.z = center.z;
  midpoint->orientation.w = q1.w();
  midpoint->orientation.x = q1.x();
  midpoint->orientation.y = q1.y();
  midpoint->orientation.z = q1.z();

  // Output dimensions.
  dimensions->x = x_length;
  dimensions->y = y_length;
  dimensions->z = z_length;
}
}  // namespace perception
}  // namespace rapid
