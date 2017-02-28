#include "rapid_perception/pose_estimation_match.h"

#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

#include "rapid_viz/markers.h"
#include "rapid_viz/point_cloud.h"
#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;

namespace rapid {
namespace perception {
PoseEstimationMatch::PoseEstimationMatch()
    : cloud_(new PointCloudC), fitness_(std::numeric_limits<double>::max()) {}

PoseEstimationMatch::PoseEstimationMatch(PointCloudC::Ptr cloud,
                                         const geometry_msgs::Pose& pose,
                                         const geometry_msgs::Vector3& scale,
                                         double fitness)
    : cloud_(new PointCloudC), pose_(pose), scale_(scale), fitness_(fitness) {
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

geometry_msgs::Vector3 PoseEstimationMatch::scale() const { return scale_; }

pcl::PointXYZ PoseEstimationMatch::center() const { return center_; }

double PoseEstimationMatch::fitness() const { return fitness_; }

void PoseEstimationMatch::set_fitness(double fitness) { fitness_ = fitness; }

bool ComparePoseEstimationMatch(const PoseEstimationMatch& a,
                                const PoseEstimationMatch& b) {
  return a.fitness() < b.fitness();
}

void VisualizeMatches(ros::Publisher& pub, ros::Publisher& marker_pub,
                      const std::vector<PoseEstimationMatch>& matches) {
  std::string frame_id = "";
  visualization_msgs::Marker clear;
  clear.ns = "output";
  clear.action = 3;
  if (marker_pub) {
    marker_pub.publish(clear);
  }
  if (pub) {
    PointCloudC::Ptr output_cloud(new PointCloudC);
    for (size_t i = 0; i < matches.size(); ++i) {
      const PoseEstimationMatch& match = matches[i];
      viz::ColorizeRandom(match.cloud());
      *output_cloud += *match.cloud();
      if (frame_id == "" && match.cloud()->header.frame_id != "") {
        frame_id = match.cloud()->header.frame_id;
      }

      if (marker_pub) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = match.cloud()->header.frame_id;
        ps.pose = match.pose();
        visualization_msgs::Marker box = viz::OutlineBox(ps, match.scale());
        box.ns = "output";
        box.id = i;
        box.scale.x = 0.0025;
        marker_pub.publish(box);
      }

      // geometry_msgs::PoseStamped ps;
      // ps.header.frame_id = match.cloud()->header.frame_id;
      // ps.pose = match.pose();
      // viz::Marker output_box =
      //    viz::Marker::OutlineBox(marker_pub, ps, object_roi_dimensions);
      // geometry_msgs::Vector3 scale;
      // scale.x = 0.0025;
      // output_box.SetScale(scale);
      // output_box.SetNamespace("output");
      // output_boxes.push_back(output_box);
      // output_boxes[output_boxes_.size() - 1].Publish();
    }
    if (frame_id == "") {
      frame_id = "base_link";
    }
    ROS_INFO("Visualizing matches in frame: %s", frame_id.c_str());
    output_cloud->header.frame_id = frame_id;
    viz::PublishCloud(pub, *output_cloud);
  }
}
}  // namespace perception
}  // namespace rapid
