#ifndef _RAPID_POSE_ESTIMATION_MATCH_H_
#define _RAPID_POSE_ESTIMATION_MATCH_H_

#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "ros/ros.h"

namespace rapid {
namespace perception {
class PoseEstimationMatch {
 public:
  // Do not use default constructor except to initialize empty objects.
  PoseEstimationMatch();
  PoseEstimationMatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                      const geometry_msgs::Pose& pose, double fitness);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud() const;
  // Returns the pose of the match.
  geometry_msgs::Pose pose() const;
  // Returns the center point of the point cloud
  pcl::PointXYZ center() const;
  // Returns the ICP fitness score (lower score is a better match).
  double fitness() const;
  void set_fitness(double fitness);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  geometry_msgs::Pose pose_;
  pcl::PointXYZ center_;
  double fitness_;
};

// Returns true if a has a lower fitness score than b.
bool ComparePoseEstimationMatch(const PoseEstimationMatch& a,
                                const PoseEstimationMatch& b);

// Visualize matches to a PointCloud2 publisher.
void VisualizeMatches(ros::Publisher& pub,
                      const std::vector<PoseEstimationMatch>& matches);
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_MATCH_H_
