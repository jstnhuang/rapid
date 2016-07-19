#include "object_search/capture_roi.h"

#include "Eigen/Core"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_msgs/CaptureRoi3D.h"
#include "rapid_msgs/Roi3D.h"
#include "rapid_perception/box3d_roi_server.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

namespace object_search {
CaptureRoi::CaptureRoi(rapid::perception::Box3DRoiServer* marker_server)
    : tf_listener_(),
      marker_server_(marker_server),
      pcl_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
      cloud_to_base_() {}

void CaptureRoi::ShowMarker() { marker_server_->Start(); }
void CaptureRoi::HideMarker() { marker_server_->Stop(); }

void CaptureRoi::set_cloud(sensor_msgs::PointCloud2ConstPtr cloud) {
  try {
    tf_listener_.lookupTransform("base_footprint", cloud->header.frame_id,
                                 cloud->header.stamp, cloud_to_base_);
  } catch (tf::TransformException e) {
    ROS_WARN("%s", e.what());
  }

  pcl::fromROSMsg(*cloud, *pcl_cloud_);
}

void CaptureRoi::Capture(sensor_msgs::PointCloud2* cloud_out) {
  pcl::CropBox<pcl::PointXYZRGB> crop_;
  crop_.setInputCloud(pcl_cloud_);

  // Get ROI
  rapid_msgs::Roi3D roi = marker_server_->roi();

  // Set cloud -> ROI rotation
  const geometry_msgs::Quaternion& roi_q = roi.transform.rotation;
  Eigen::Quaternionf roi_rot(roi_q.w, roi_q.x, roi_q.y, roi_q.z);
  const tf::Quaternion& cloud_q = cloud_to_base_.getRotation();
  Eigen::Quaternionf cloud_rot(cloud_q.w(), cloud_q.x(), cloud_q.y(),
                               cloud_q.z());
  Eigen::Affine3f transform((cloud_rot * roi_rot).toRotationMatrix());
  crop_.setTransform(transform);

  // Set cloud -> ROI translation
  const tf::Vector3& cloud_t = -1 * cloud_to_base_.getOrigin();
  const geometry_msgs::Vector3& roi_t = roi.transform.translation;
  Eigen::Vector3f translation(cloud_t.x() + roi_t.x, cloud_t.y() + roi_t.y,
                              cloud_t.z() + roi_t.z);
  crop_.setTranslation(translation);

  // Set dimensions
  Eigen::Vector4f min_pt(-roi.dimensions.x / 2, -roi.dimensions.y / 2,
                         -roi.dimensions.z / 2, 0);
  crop_.setMin(min_pt);
  Eigen::Vector4f max_pt(roi.dimensions.x / 2, roi.dimensions.y / 2,
                         roi.dimensions.z / 2, 0);
  crop_.setMax(max_pt);
  pcl::PointCloud<pcl::PointXYZRGB> output;
  crop_.filter(output);

  ROS_INFO("Captured cloud with %ld points", output.size());

  pcl::toROSMsg(output, *cloud_out);
}

tf::StampedTransform CaptureRoi::cloud_to_base() { return cloud_to_base_; }
}  // namespace object_search
