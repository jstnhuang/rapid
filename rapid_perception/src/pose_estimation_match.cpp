#include "rapid_perception/pose_estimation_match.h"

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;

namespace rapid {
namespace perception {
PoseEstimationMatch::PoseEstimationMatch()
    : cloud_(new PointCloudC), fitness_(std::numeric_limits<double>::max()) {}

PoseEstimationMatch::PoseEstimationMatch(PointCloudC::Ptr cloud,
                                         const geometry_msgs::Pose& pose,
                                         double fitness)
    : cloud_(new PointCloudC), pose_(pose), fitness_(fitness) {
  *cloud_ = *cloud;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*cloud_, min_pt, max_pt);
  center_.x = min_pt.x() + (max_pt.x() - min_pt.x()) / 2;
  center_.y = min_pt.y() + (max_pt.y() - min_pt.y()) / 2;
  center_.z = min_pt.z() + (max_pt.z() - min_pt.z()) / 2;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PoseEstimationMatch::cloud() const {
  return cloud_;
}

geometry_msgs::Pose PoseEstimationMatch::pose() const { return pose_; }

pcl::PointXYZ PoseEstimationMatch::center() const { return center_; }

double PoseEstimationMatch::fitness() const { return fitness_; }

void PoseEstimationMatch::set_fitness(double fitness) { fitness_ = fitness; }

bool ComparePoseEstimationMatch(const PoseEstimationMatch& a,
                                const PoseEstimationMatch& b) {
  return a.fitness() < b.fitness();
}
}  // namespace perception
}  // namespace rapid
