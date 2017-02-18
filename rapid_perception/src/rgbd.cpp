#include "rapid_perception/rgbd.h"

#include <algorithm>
#include <math.h>

#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "pcl/common/pca.h"
#include "pcl/filters/random_sample.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/search/kdtree.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_perception/conversions.h"

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;

typedef PointXYZRGB PointC;
typedef PointCloud<PointXYZRGB> PointCloudC;

namespace rapid {
namespace perception {
bool FindHorizontalPlane(const pcl::PointCloud<PointXYZRGB>::ConstPtr& cloud,
                         double distance_threshold, double eps_angle,
                         PointIndices::Ptr inliers) {
  PointIndices::Ptr pi;
  return FindHorizontalPlane(cloud, pi, distance_threshold, eps_angle, inliers);
}

bool FindHorizontalPlane(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
                         const PointIndices::ConstPtr& indices,
                         double distance_threshold, double eps_angle,
                         PointIndices::Ptr inliers) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(eps_angle);

  seg.setInputCloud(cloud);
  if (indices) {
    seg.setIndices(indices);
  }
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    return false;
  }
  return true;
}

void ClosestAxis(const Eigen::Matrix3d& mat, const Eigen::Vector3d& vec,
                 int* index, double* cosine_sim) {
  double sim = 0;
  double best_sim = 0;  // Highest absolute value of cosine sim.

  sim = mat.col(0).dot(vec) / mat.col(0).norm() * vec.norm();
  best_sim = fabs(sim);
  *index = 0;

  sim = mat.col(1).dot(vec) / mat.col(1).norm() * vec.norm();
  if (fabs(sim) > best_sim) {
    best_sim = fabs(sim);
    *cosine_sim = sim;
    *index = 1;
  }

  sim = mat.col(2).dot(vec) / mat.col(2).norm() * vec.norm();
  if (fabs(sim) > best_sim) {
    best_sim = fabs(sim);
    *cosine_sim = sim;
    *index = 2;
  }
}

void GetBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_in,
                    const pcl::PointIndices::ConstPtr& indices,
                    geometry_msgs::PoseStamped* midpoint,
                    geometry_msgs::Vector3* dimensions) {
  // We reify the input cloud because PCA uses the entire cloud for projection.
  PointCloud<PointXYZRGB>::Ptr cloud = IndicesToCloud(cloud_in, indices);
  // Compute PCA basis
  pcl::PCA<PointXYZRGB> pca(/* basis_only */ true);
  pca.setInputCloud(cloud);
  Eigen::Matrix3d eigenvectors = pca.getEigenVectors().cast<double>();

  // Project to eigenspace
  PointCloud<PointXYZRGB>::Ptr eigen_projected(new PointCloud<PointXYZRGB>);
  pca.project(*cloud, *eigen_projected);

  // Compute dimensions
  PointXYZRGB eigen_min;
  PointXYZRGB eigen_max;
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

  // Flip things around so +z and +x line up with the cloud.
  double z_cosine_sim = 0;
  int z_closest_index = 0;
  ClosestAxis(eigenvectors, Eigen::Vector3d::UnitZ(), &z_closest_index,
              &z_cosine_sim);
  double x_cosine_sim = 0;
  int x_closest_index = 0;
  ClosestAxis(eigenvectors, Eigen::Vector3d::UnitX(), &x_closest_index,
              &x_cosine_sim);

  Eigen::Matrix3d mygenvectors;
  mygenvectors.col(0) = eigenvectors.col(x_closest_index);
  if (x_cosine_sim < 0) {
    mygenvectors.col(0) *= -1;
  }
  mygenvectors.col(2) = eigenvectors.col(z_closest_index);
  if (z_cosine_sim < 0) {
    mygenvectors.col(2) *= -1;
  }
  mygenvectors.col(1) = mygenvectors.col(2).cross(mygenvectors.col(0));

  if (z_closest_index == 0) {
    std::swap(x_length, z_length);
  } else if (z_closest_index == 1) {
    std::swap(y_length, z_length);
  }

  if (x_closest_index == 1) {
    std::swap(x_length, y_length);
  } else if (x_closest_index == 2) {
    std::swap(x_length, z_length);
  }

  Eigen::Quaterniond q1(mygenvectors);

  // Output midpoint.
  midpoint->header.frame_id = cloud->header.frame_id;
  midpoint->pose.position.x = center.x;
  midpoint->pose.position.y = center.y;
  midpoint->pose.position.z = center.z;
  midpoint->pose.orientation.w = q1.w();
  midpoint->pose.orientation.x = q1.x();
  midpoint->pose.orientation.y = q1.y();
  midpoint->pose.orientation.z = q1.z();

  // Output dimensions.
  dimensions->x = x_length;
  dimensions->y = y_length;
  dimensions->z = z_length;
}

void GetPlanarBoundingBox(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
                          const PointIndices::ConstPtr& indices,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  // original_cloud stores the cloud subset we are using.
  PointCloud<PointXYZRGB>::Ptr original_cloud;
  if (indices) {
    original_cloud = IndicesToCloud(cloud, indices);
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

  PointXYZRGB eigen_min;
  PointXYZRGB eigen_max;
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

void GetPlanarBoundingBox(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  PointIndices::Ptr pi;
  GetPlanarBoundingBox(cloud, pi, midpoint, dimensions);
}

PointCloud<PointXYZRGB>::Ptr IndicesToCloud(
    const PointCloud<PointXYZRGB>::ConstPtr& cloud,
    const PointIndices::ConstPtr& indices) {
  PointCloud<PointXYZRGB>::Ptr result(new PointCloud<PointXYZRGB>());
  pcl::ExtractIndices<PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*result);
  return result;
}

double ComputeResolution(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
  pcl::search::KdTree<PointXYZRGB> tree;
  tree.setInputCloud(cloud);
  return ComputeResolution(cloud, tree);
}

double ComputeResolution(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const pcl::search::Search<pcl::PointXYZRGB>& tree) {
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);

  for (size_t i = 0; i < cloud->size(); ++i) {
    if (!pcl_isfinite((*cloud)[i].x)) {
      continue;
    }
    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2) {
      res += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0) {
    res /= n_points;
  }
  return res;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Average(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds) {
  PointCloud<PointXYZRGB>::Ptr input(new PointCloud<PointXYZRGB>);
  for (size_t i = 0; i < clouds.size(); ++i) {
    *input += *clouds[i];
  }

  double random_sample_factor;
  ros::param::param("random_sample_factor", random_sample_factor, 1.0);
  pcl::RandomSample<PointC> random;
  random.setInputCloud(input);
  int num_points =
      static_cast<int>(random_sample_factor * input->size() / clouds.size());
  random.setSample(num_points);
  PointCloud<PointXYZRGB>::Ptr sampled(new PointCloud<PointXYZRGB>);
  random.filter(*sampled);

  int sor_mean_k;
  double sor_stddev;
  ros::param::param("sor_mean_k", sor_mean_k, 50);
  ros::param::param("sor_stddev", sor_stddev, 1.0);
  pcl::StatisticalOutlierRemoval<PointC> sor;
  sor.setInputCloud(sampled);
  sor.setMeanK(sor_mean_k);
  sor.setStddevMulThresh(sor_stddev);
  PointCloudC::Ptr filtered(new PointCloudC);
  sor.filter(*filtered);

  filtered->header.frame_id = clouds[0]->header.frame_id;
  ROS_INFO("%ld points in average", filtered->size());
  return filtered;
}

PointCloud<PointXYZRGB>::Ptr GetSmoothedKinectCloud(const std::string& topic,
                                                    int num_clouds) {
  std::vector<PointCloudC::Ptr> clouds;
  for (int i = 0; i < num_clouds; ++i) {
    sensor_msgs::PointCloud2ConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic);
    ROS_INFO("Read cloud %d of %d", i + 1, num_clouds);
    // PointCloudC::Ptr cloud = PclFromRos(*msg);
    clouds.push_back(PclFromRos(*msg));
  }

  return Average(clouds);
}
}  // namespace perception
}  // namespace rapid
