#include "pcl/filters/crop_box.h"
#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "rapid_perception/box3d_roi_server.h"
#include "sensor_msgs/PointCloud2.h"
#include "rapid_msgs/CaptureRoi3D.h"
#include "rapid_msgs/Roi3D.h"
#include "tf/transform_listener.h"

#include <Eigen/Dense>

class CaptureRoi {
 public:
  CaptureRoi();
  void Start();
  void set_cloud(sensor_msgs::PointCloud2ConstPtr cloud);
  void set_roi(const rapid_msgs::Roi3D& roi);
  sensor_msgs::PointCloud2 Capture();
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool ServeCapture(rapid_msgs::CaptureRoi3DRequest& req,
                    rapid_msgs::CaptureRoi3DResponse& res);

 private:
  ros::NodeHandle nh_;
  tf::StampedTransform cloud_to_base_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
  rapid_msgs::Roi3D roi_;
  rapid::perception::Box3DRoiServer marker_server_;
  tf::TransformListener tf_listener_;
  ros::Publisher output_pub_;
};

CaptureRoi::CaptureRoi()
    : nh_(),
      cloud_to_base_(),
      pcl_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
      roi_(),
      marker_server_("roi1"),
      tf_listener_(),
      output_pub_(
          nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, true)) {}

void CaptureRoi::Start() { marker_server_.Start(); }

void CaptureRoi::set_cloud(sensor_msgs::PointCloud2ConstPtr cloud) {
  pcl::fromROSMsg(*cloud, *pcl_cloud_);
}

void CaptureRoi::set_roi(const rapid_msgs::Roi3D& roi) { roi_ = roi; }

sensor_msgs::PointCloud2 CaptureRoi::Capture() {
  pcl::CropBox<pcl::PointXYZRGB> crop_;
  crop_.setInputCloud(pcl_cloud_);

  // Set cloud -> ROI rotation
  const geometry_msgs::Quaternion& roi_q = roi_.transform.rotation;
  Eigen::Quaternionf roi_rot(roi_q.w, roi_q.x, roi_q.y, roi_q.z);
  const tf::Quaternion& cloud_q = cloud_to_base_.getRotation();
  Eigen::Quaternionf cloud_rot(cloud_q.w(), cloud_q.x(), cloud_q.y(),
                               cloud_q.z());
  Eigen::Affine3f transform((cloud_rot * roi_rot).toRotationMatrix());
  crop_.setTransform(transform);

  // Set cloud -> ROI translation
  const tf::Vector3& cloud_t = -1 * cloud_to_base_.getOrigin();
  const geometry_msgs::Vector3& roi_t = roi_.transform.translation;
  Eigen::Vector3f translation(cloud_t.x() + roi_t.x, cloud_t.y() + roi_t.y,
                              cloud_t.z() + roi_t.z);
  crop_.setTranslation(translation);

  // Set dimensions
  Eigen::Vector4f min_pt(-roi_.dimensions.x / 2, -roi_.dimensions.y / 2,
                         -roi_.dimensions.z / 2, 0);
  crop_.setMin(min_pt);
  Eigen::Vector4f max_pt(roi_.dimensions.x / 2, roi_.dimensions.y / 2,
                         roi_.dimensions.z / 2, 0);
  crop_.setMax(max_pt);
  pcl::PointCloud<pcl::PointXYZRGB> output;
  crop_.filter(output);
  sensor_msgs::PointCloud2 output_ros;
  pcl::toROSMsg(output, output_ros);
  return output_ros;
}

void CaptureRoi::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  try {
    tf_listener_.lookupTransform("base_link", msg->header.frame_id,
                                 msg->header.stamp, cloud_to_base_);
    set_cloud(msg);
  } catch (tf::TransformException e) {
  }
}

bool CaptureRoi::ServeCapture(rapid_msgs::CaptureRoi3DRequest& req,
                              rapid_msgs::CaptureRoi3DResponse& res) {
  roi_ = marker_server_.roi();
  const sensor_msgs::PointCloud2& output = Capture();
  output_pub_.publish(output);
  res.cloud = Capture();

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "roi3d_main");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::NodeHandle nh;
  CaptureRoi capture_server;
  ros::Subscriber pc_sub =
      nh.subscribe("cloud_in", 1, &CaptureRoi::CloudCallback, &capture_server);
  ros::ServiceServer save_srv = nh.advertiseService(
      "capture_roi", &CaptureRoi::ServeCapture, &capture_server);
  capture_server.Start();
  ros::waitForShutdown();
  return 0;
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) { return; }
bool capture_roi(rapid_msgs::CaptureRoi3DRequest& req,
                 rapid_msgs::CaptureRoi3DResponse& res) {
  return true;
}
