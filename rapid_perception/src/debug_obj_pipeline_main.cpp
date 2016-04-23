#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "boost/algorithm/string.hpp"
#include "boost/shared_ptr.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#include "rapid_perception/pr2.h"
#include "rapid_perception/rgbd.hpp"
#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"
#include "rapid_utils/math.h"

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using pcl::PointCloud;
using pcl::PointXYZRGB;

namespace rpe = rapid::perception;

class Perception {
 public:
  Perception()
      : nh_(),
        tf_listener_(),
        pcl_cloud_(new PointCloud<PointXYZRGB>),
        cloud_pub_(
            nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1, false)),
        vis_pub_(
            nh_.advertise<visualization_msgs::Marker>("debug_vis", 1, false)),
        scene_() {}

  void set_cloud(const sensor_msgs::PointCloud2& cloud) {
    sensor_msgs::PointCloud2 transformed;
    tf_listener_.waitForTransform("/base_footprint", cloud.header.frame_id,
                                  ros::Time(0), ros::Duration(10));
    pcl_ros::transformPointCloud("/base_footprint", cloud, transformed,
                                 tf_listener_);
    pcl::fromROSMsg(transformed, *pcl_cloud_);
  }

  void Crop() {
    Eigen::Vector4f min;
    rpe::ParseParams params = rpe::Pr2Params();
    min << params.scene.min_x, params.scene.min_y, params.scene.min_z, 1;
    Eigen::Vector4f max;
    max << params.scene.max_x, params.scene.max_y, params.scene.max_z, 1;

    pcl::CropBox<PointXYZRGB> cb;
    cb.setInputCloud(pcl_cloud_);
    cb.setMin(min);
    cb.setMax(max);
    cb.filter(*pcl_cloud_);
  }

  void FindPlane() {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    rpe::FindHorizontalPlane<PointXYZRGB>(
        pcl_cloud_, 0.01, rapid::utils::DegreesToRadians(5), inliers);
    PointCloud<PointXYZRGB>::Ptr plane =
        rpe::IndicesToCloud<PointXYZRGB>(pcl_cloud_, inliers);
    *pcl_cloud_ = *plane;
  }

  void Colorize(PointCloud<PointXYZRGB>& cloud, int r, int g, int b,
                double alpha) {
    for (size_t i = 0; i < cloud.size(); ++i) {
      PointXYZRGB& point = cloud[i];
      point.r =
          std::min(255, static_cast<int>((1 - alpha) * point.r + (alpha * r)));
      point.g =
          std::min(255, static_cast<int>((1 - alpha) * point.g + (alpha * g)));
      point.b =
          std::min(255, static_cast<int>((1 - alpha) * point.b + (alpha * b)));
    }
  }

  void ParseScene() {
    bool success = rpe::ParseScene(pcl_cloud_, rpe::Pr2Params(), &scene_);
    if (!success) {
      ROS_ERROR("Failed to parse scene.");
      return;
    }
    const rpe::HSurface& tt = scene_.primary_surface();
    const vector<rpe::Object>& objects = tt.objects();
    PointCloud<PointXYZRGB>::Ptr table_cloud = tt.GetCloud();
    PointCloud<PointXYZRGB>::Ptr display(new PointCloud<PointXYZRGB>);
    display->header.frame_id = "base_footprint";
    *display = *table_cloud;

    cout << "Found " << objects.size() << " objects." << endl;
    for (size_t j = 0; j < objects.size(); ++j) {
      const rpe::Object& obj = objects[j];
      PointCloud<PointXYZRGB>::Ptr obj_cloud = obj.GetCloud();
      int r = std::rand() % 255;
      int g = std::rand() % 255;
      int b = std::rand() % 255;
      Colorize(*obj_cloud, r, g, b, 0.5);
      *display += *obj_cloud;
    }
    *pcl_cloud_ = *display;
    // scene_.Visualize();
  }

  void PublishCloud() {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*pcl_cloud_, cloud);
    cloud.header.stamp = ros::Time::now();
    cloud_pub_.publish(cloud);
  }

  void Visualize() {
    // scene_.Visualize();
  }

 private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  PointCloud<PointXYZRGB>::Ptr pcl_cloud_;
  ros::Publisher cloud_pub_;
  ros::Publisher vis_pub_;
  rpe::Scene scene_;
};

class Interpreter {
 public:
  Interpreter() : perception_() {}

  void PrintCommands() {
    cout << "Commands:" << endl;
    cout << "  read: Reads in a new point cloud." << endl;
    cout << "  crop: Crops the point cloud with default params." << endl;
    cout << "  find_plane: Finds the plane in the current cloud." << endl;
    cout << "  parse_scene: Parses the scene." << endl;
    cout << "  exit: Exits the app." << endl;
  }

  bool ParseCommand(const string& input, string* command,
                    vector<string>* args) {
    string trimmed = boost::trim_copy(input);
    if (input == "read") {
      *command = "read";
    } else if (input == "crop") {
      *command = "crop";
    } else if (input == "find_plane") {
      *command = "find_plane";
    } else if (input == "parse_scene") {
      *command = "parse_scene";
    } else if (input == "viz") {
      *command = "viz";
    } else if (input == "exit") {
      *command = "exit";
    } else {
      return false;
    }
    return true;
  }

  void GetCommand(string* command, vector<string>* args) {
    args->clear();
    bool success = false;
    while (!success) {
      PrintCommands();
      string input;
      cout << "Enter command: ";
      cin >> input;
      cout << endl;
      success = ParseCommand(input, command, args);
    }
  }

  void RunCommand(const string& command, const vector<string>& args) {
    if (command == "read") {
      sensor_msgs::PointCloud2ConstPtr msg =
          ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cloud_in");
      perception_.set_cloud(*msg);
      perception_.PublishCloud();
    } else if (command == "crop") {
      perception_.Crop();
      perception_.PublishCloud();
    } else if (command == "find_plane") {
      perception_.FindPlane();
      perception_.PublishCloud();
    } else if (command == "parse_scene") {
      perception_.ParseScene();
      perception_.PublishCloud();
    } else if (command == "viz") {
      perception_.Visualize();
    } else {
    }
  }

 private:
  Perception perception_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "debug_obj_pipeline");
  ros::NodeHandle nh;
  Interpreter interpreter;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  while (true) {
    string command;
    vector<string> args;
    interpreter.GetCommand(&command, &args);
    if (command == "exit") {
      break;
    }
    interpreter.RunCommand(command, args);
  }
  spinner.stop();
  return 0;
}
