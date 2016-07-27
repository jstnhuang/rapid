#ifndef _RAPID_PERCEPTION_IMAGE_RECOGNITION_H_
#define _RAPID_PERCEPTION_IMAGE_RECOGNITION_H_

#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "boost/shared_ptr.hpp"
#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/net.hpp"
#include "caffe/proto/caffe.pb.h"
#include "opencv2/opencv.hpp"

namespace rapid {
namespace perception {
// Runs a Caffe CNN model on an image.
//
// Usage:
//   string error(""); // Will be filled out on error.
//   ImageRecognizer recognizer;
//   ImageRecognizer::AlexNet("/home/user/alexnet", &recognizer, &error);
//   cv::Mat image; // Input is an OpenCV image.
//   recognizer.set_image(image);
//   cv::Mat conv3 = recognizer.layer("conv3", &error);
//   cv::Mat fc6 = recognizer.layer("fc6", &error);
//   vector<std::pair<string, float>> predictions = recognizer.predictions(5,
//    &error);
class ImageRecognizer {
 public:
  ImageRecognizer();  // Do not use, call ImageRecognizer::AlexNet() instead.
  ImageRecognizer(boost::shared_ptr<caffe::Net<float> > feature_extraction_net,
                  cv::Mat mean, cv::Size input_geometry, int num_channels,
                  const std::vector<std::string>& labels);
  void set_image(const cv::Mat& image);
  // Get a layer as a single vector
  cv::Mat layer(const std::string& layer_name, std::string* error);
  std::vector<std::pair<std::string, float> > predictions(int num_predictions,
                                                          std::string* error);
  static bool AlexNet(std::string model_dir, ImageRecognizer* recognizer,
                      std::string* error);

 private:
  void ForwardPass(std::string* error);
  std::vector<float> Predict(const cv::Mat& image, std::string* error);
  void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels,
                  std::string* error);
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

  static bool BuildFromFiles(const std::string& model_prototxt,
                             const std::string& pretrained_caffemodel,
                             const std::string& mean_binaryproto,
                             const std::string& labels_file,
                             ImageRecognizer* recognizer, std::string* error);

  boost::shared_ptr<caffe::Net<float> > net_;
  cv::Mat image_;
  cv::Mat mean_;  // Mean of the training dataset
  cv::Size input_geometry_;
  int num_channels_;
  std::vector<std::string> labels_;

  bool needs_update_;  // True if new image.
};

cv::Mat ReadMeanFile(const std::string& mean_file,
                     const cv::Size& input_geometry, int num_channels,
                     std::string* error = NULL);

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs);

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N);
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_IMAGE_RECOGNITION_H_
