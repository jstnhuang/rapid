#include "rapid_perception/pose_estimation_cnn_heat_mapper.h"

#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/core/core.hpp"
#include "pcl/PointIndices.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"

#include "rapid_perception/cloud_projection.h"
#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::search::KdTree<PointC> PointCTree;

using std::string;
using std::vector;

namespace rapid {
namespace perception {
CnnHeatMapper::CnnHeatMapper()
    : scene_(new PointCloudC()),
      scene_camera_(new PointCloudC()),
      scene_tree_(),
      object_(new PointCloudC()),
      object_camera_(new PointCloudC()),
      object_radius_est_(0),
      object_image_(),
      image_recognizer_(),
      sample_ratio_(0.01),
      max_samples_(2000),
      max_sample_radius_(0.05),
      cnn_layer_("fc6"),
      landmark_image_pub_(),
      scene_image_pub_() {}

void CnnHeatMapper::Compute(PointCloudC::Ptr heatmap,
                            Eigen::VectorXd* importances) {
  pcl::PointIndicesPtr indices(new pcl::PointIndices);
  // Sample points in the scene.
  pcl::RandomSample<PointC> random;
  random.setInputCloud(scene_);
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  random.setSample(std::min(max_samples_, num_samples));
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  // For each sample, look at the points around it, project the image into 2D,
  // and use CNN features to compare to the landmark.
  double search_radius = std::min(max_sample_radius_, object_radius_est_);
  importances->resize(indices->indices.size());
  pcl::PointIndicesPtr k_indices_ptr(new pcl::PointIndices);
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int index = indices->indices[indices_i];
    vector<int> k_indices;
    vector<float> k_distances;
    scene_tree_->radiusSearch(index, search_radius, k_indices, k_distances,
                              object_->size());
    k_indices_ptr->indices = k_indices;
    cv::Mat scene_image;
    CloudToImage(HeadMountKinectCameraInfo(), *scene_camera_, k_indices_ptr,
                 &scene_image);
    viz::PublishImage(scene_image_pub_, scene_camera_->header, scene_image);

    // Paste the landmark into the scene
    double width_ratio =
        object_image_.cols / static_cast<double>(scene_image.cols);
    double height_ratio =
        object_image_.rows / static_cast<double>(scene_image.rows);
    double max_ratio = std::max(width_ratio, height_ratio);
    cv::Mat object_resized;
    if (max_ratio > 1) {
      double ratio = 1 / max_ratio;
      cv::resize(object_image_, object_resized, cv::Size(), ratio, ratio);
    } else {
      object_resized = object_image_;
    }
    // Paste into the center
    cv::Mat landmark_pasted = scene_image.clone();
    int row_offset = (landmark_pasted.rows - object_resized.rows) / 2;
    int col_offset = (landmark_pasted.cols - object_resized.cols) / 2;
    for (int row = 0; row < object_resized.rows; ++row) {
      uchar* obj_p = object_resized.ptr<uchar>(row);
      uchar* pasted_p = landmark_pasted.ptr<uchar>(row + row_offset);
      for (int col = 0; col < object_resized.cols; ++col) {
        bool is_black = true;
        for (int color = 0; color < 3; ++color) {
          if (obj_p[3 * col + color] != 0) {
            is_black = false;
            break;
          }
        }
        if (!is_black) {
          for (int color = 0; color < 3; ++color) {
            pasted_p[3 * (col + col_offset) + color] = obj_p[3 * col + color];
          }
        }
      }
    }
    viz::PublishImage(landmark_image_pub_, object_camera_->header,
                      landmark_pasted);

    image_recognizer_.set_image(landmark_pasted);
    string error;
    cv::Mat pasted_features = image_recognizer_.layer(cnn_layer_, &error);
    if (error != "") {
      ROS_ERROR("Error getting object image: %s", error.c_str());
    }

    image_recognizer_.set_image(scene_image);
    error = "";
    cv::Mat scene_features = image_recognizer_.layer(cnn_layer_, &error);
    if (error != "") {
      ROS_ERROR("Error getting scene image: %s", error.c_str());
    }
    double cosine_sim = scene_features.dot(pasted_features) /
                        (cv::norm(scene_features) * cv::norm(pasted_features));
    (*importances)(indices_i) = cosine_sim;
    if (debug_) {
      PointCloudC::Ptr debug(new PointCloudC);
      const PointC& current = scene_->at(index);
      PointC current_copy = current;
      current_copy.r = 255;
      current_copy.g = 0;
      current_copy.b = 0;
      debug->push_back(current_copy);
      debug->header.frame_id = scene_->header.frame_id;
      viz::PublishCloud(heatmap_pub_, *debug);
      std::cout << "# scene points: " << k_indices.size()
                << ", cosine sim: " << cosine_sim;
      string input;
      std::getline(std::cin, input);
    }
  }

  double min = importances->minCoeff();
  if (min < 0) {
    ROS_ERROR("Min cosine distance was negative");
  }
  (*importances) = (importances->array() - min).matrix();
  double max = importances->maxCoeff();
  (*importances) /= max;

  // Do softmax on top of normalizing between 0 and 1
  (*importances) = importances->array().exp().matrix();

  // Color point cloud for visualization
  PointCloudC::Ptr working_scene(new PointCloudC);
  *working_scene = *scene_;
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int color = static_cast<int>(round(255 * (*importances)(indices_i)));
    int index = indices->indices[indices_i];
    working_scene->points[index].r = color;
    working_scene->points[index].g = color;
    working_scene->points[index].b = color;
  }
  PointCloudC::Ptr viz_cloud(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(working_scene);
  extract.setIndices(indices);
  extract.filter(*viz_cloud);
  viz::PublishCloud(heatmap_pub_, *viz_cloud);
  heatmap = viz_cloud;
}

void CnnHeatMapper::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
  scene_tree_.reset(new PointCTree());
  scene_tree_->setInputCloud(scene_);
}

void CnnHeatMapper::set_scene_camera(PointCloudC::Ptr scene) {
  scene_camera_ = scene;
}

void CnnHeatMapper::set_object(PointCloudC::Ptr object) {
  object_ = object;

  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  double x = max_pt.x() - min_pt.x();
  double y = max_pt.y() - min_pt.y();
  double z = max_pt.z() - min_pt.z();
  object_radius_est_ = std::max(x, std::max(y, z));
}

void CnnHeatMapper::set_object_camera(PointCloudC::Ptr object) {
  object_camera_ = object;
  CloudToImage(HeadMountKinectCameraInfo(), *object_camera_, &object_image_);
  viz::PublishImage(landmark_image_pub_, object_camera_->header, object_image_);
}

void CnnHeatMapper::set_image_recognizer(const ImageRecognizer& val) {
  image_recognizer_ = val;
}

void CnnHeatMapper::set_sample_ratio(double val) { sample_ratio_ = val; }
void CnnHeatMapper::set_max_samples(int val) { max_samples_ = val; }
void CnnHeatMapper::set_max_sample_radius(double val) {
  max_sample_radius_ = val;
}
void CnnHeatMapper::set_cnn_layer(string val) { cnn_layer_ = val; }

void CnnHeatMapper::set_landmark_image_publisher(const ros::Publisher& pub) {
  landmark_image_pub_ = pub;
}
void CnnHeatMapper::set_scene_image_publisher(const ros::Publisher& pub) {
  scene_image_pub_ = pub;
}
}  // namespace perception
}  // namespace rapid
