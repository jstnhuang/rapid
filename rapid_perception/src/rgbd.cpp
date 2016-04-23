#include "rapid_perception/rgbd.hpp"

#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "pcl/common/pca.h"

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;

namespace rapid {
namespace perception {
void GetPlanarBoundingBox(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
                          const PointIndices::ConstPtr& indices,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  // original_cloud stores the cloud subset we are using.
  PointCloud<PointXYZRGB>::Ptr original_cloud;
  if (indices) {
    original_cloud = IndicesToCloud<PointXYZRGB>(cloud, indices);
  } else {
    original_cloud.reset(new PointCloud<PointXYZRGB>);
    *original_cloud = *cloud;
  }

  // Project points onto XY plane. Need to make a copy because we need the
  // original z values later.
  PointCloud<PointXYZRGB>::Ptr working(
      new PointCloud<PointXYZRGB>(*original_cloud));
  for (size_t i = 0; i < working->points.size(); ++i) {
    working->points[i].z = 0;
  }

  // Compute PCA.
  pcl::PCA<PointXYZRGB> pca(/* basis_only */ true);
  pca.setInputCloud(working);

  // Get eigenvectors.
  Eigen::Matrix3d eigenvectors = pca.getEigenVectors().cast<double>();
  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

  Eigen::Quaterniond q1(eigenvectors);
  if (eigenvectors(2, 2) < 0) {  // z-axis is pointing down
    Eigen::Quaterniond roll_180;
    roll_180 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    q1 *= roll_180;
  }

  // Find min/max x and y, based on the points in eigenspace.
  PointCloud<PointXYZRGB>::Ptr eigen_projected(new PointCloud<PointXYZRGB>);
  pca.project(*original_cloud, *eigen_projected);

  pcl::PointXYZRGB eigen_min;
  pcl::PointXYZRGB eigen_max;
  pcl::getMinMax3D(*eigen_projected, eigen_min, eigen_max);
  double x_length = eigen_max.x - eigen_min.x;
  double y_length = eigen_max.y - eigen_min.y;
  double z_length = eigen_max.z - eigen_min.z;

  // Compute midpoint, defined as the midpoint between the minimum and maximum
  // points, in x, y, and z directions. The centroid is an average that depends
  // on the density of points, which doesn't give you the geometric center of
  // the point cloud.
  PointXYZRGB eigen_center;
  eigen_center.x = eigen_min.x + x_length / 2;
  eigen_center.y = eigen_min.y + y_length / 2;
  eigen_center.z = eigen_min.z + z_length / 2;
  PointXYZRGB center;
  pca.reconstruct(eigen_center, center);

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

void GetPlanarBoundingBox(const PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  PointIndices::Ptr pi;
  GetPlanarBoundingBox(cloud, pi, midpoint, dimensions);
}
}  // namespace perception
}  // namespace rapid
