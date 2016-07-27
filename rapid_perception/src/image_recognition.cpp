#include "rapid_perception/image_recognition.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "boost/shared_ptr.hpp"
#include "caffe/caffe.hpp"
#include "caffe/net.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using boost::shared_ptr;
using std::pair;
using std::string;
using std::vector;

namespace rapid {
namespace perception {
// Do not use default constructor except when calling AlexNet().
ImageRecognizer::ImageRecognizer() {}

ImageRecognizer::ImageRecognizer(boost::shared_ptr<caffe::Net<float> > net,
                                 cv::Mat mean, cv::Size input_geometry,
                                 int num_channels, const vector<string>& labels)
    : net_(net),
      mean_(mean),
      input_geometry_(input_geometry),
      num_channels_(num_channels),
      labels_(labels),
      needs_update_(false) {
#ifdef CPU_ONLY
  caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
  caffe::Caffe::SetDevice(0);
#endif
}

void ImageRecognizer::set_image(const cv::Mat& image) {
  image_ = image;
  needs_update_ = true;
}

cv::Mat ImageRecognizer::layer(const string& layer_name, string* error) {
  if (!net_->has_blob(layer_name)) {
    *error = "CNN has no layer named " + layer_name;
    cv::Mat mat;
    return mat;
  }
  if (needs_update_) {
    ForwardPass(error);
    if (*error != "") {
      cv::Mat mat;
      return mat;
    }
  }
  const boost::shared_ptr<caffe::Blob<float> > feature_blob =
      net_->blob_by_name(layer_name);

  // caffe::Datum datum;
  // datum.set_height(feature_blob->height());
  // datum.set_width(feature_blob->width());
  // datum.set_channels(feature_blob->channels());
  // datum.clear_data();
  // datum.clear_float_data();
  const float* feature_blob_data = feature_blob->cpu_data();

  int dim_features = feature_blob->count();
  cv::Mat mat(dim_features, 1, CV_32F);
  float* mat_p = mat.ptr<float>(0);
  for (int d = 0; d < dim_features; ++d) {
    // datum.add_float_data(feature_blob_data[d]);
    mat_p[d] = feature_blob_data[d];
  }
  return mat;
}

vector<pair<string, float> > ImageRecognizer::predictions(int num_predictions,
                                                          string* error) {
  vector<pair<string, float> > predictions;
  std::vector<float> output = Predict(image_, error);
  if (*error != "") {
    return predictions;
  }

  num_predictions = std::min<int>(labels_.size(), num_predictions);
  vector<int> maxN = Argmax(output, num_predictions);
  for (int i = 0; i < num_predictions; ++i) {
    int idx = maxN[i];
    predictions.push_back(std::make_pair(labels_[idx], output[idx]));
  }

  return predictions;
}

void ImageRecognizer::ForwardPass(string* error) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_, input_geometry_.height,
                       input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(image_, &input_channels, error);
  if (*error != "") {
    return;
  }

  net_->Forward();
  needs_update_ = false;
}

vector<float> ImageRecognizer::Predict(const cv::Mat& image, string* error) {
  if (needs_update_) {
    ForwardPass(error);
    if (*error != "") {
      vector<float> empty;
      return empty;
    }
  }

  /* Copy the output layer to a std::vector */
  caffe::Blob<float>* output_layer = net_->output_blobs()[0];
  const float* begin = output_layer->cpu_data();
  const float* end = begin + output_layer->channels();
  return std::vector<float>(begin, end);
}

void ImageRecognizer::Preprocess(const cv::Mat& img,
                                 std::vector<cv::Mat>* input_channels,
                                 string* error) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1) {
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  } else if (img.channels() == 4 && num_channels_ == 1) {
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  } else if (img.channels() == 4 && num_channels_ == 3) {
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  } else if (img.channels() == 1 && num_channels_ == 3) {
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  } else {
    sample = img;
  }

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_) {
    cv::resize(sample, sample_resized, input_geometry_);
  } else {
    sample_resized = sample;
  }

  cv::Mat sample_float;
  if (num_channels_ == 3) {
    sample_resized.convertTo(sample_float, CV_32FC3);
  } else {
    sample_resized.convertTo(sample_float, CV_32FC1);
  }

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  if (reinterpret_cast<float*>(input_channels->at(0).data) !=
      net_->input_blobs()[0]->cpu_data()) {
    if (error != NULL) {
      *error =
          "Input channels are not wrapping the input layer of the network.";
    }
    return;
  }
}

void ImageRecognizer::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

bool ImageRecognizer::AlexNet(std::string model_dir,
                              ImageRecognizer* recognizer, string* error) {
  if (model_dir[model_dir.length() - 1] != '/') {
    model_dir += "/";
  }
  string model_prototxt = model_dir + "deploy.prototxt";
  string pretrained_caffemodel =
      model_dir + "bvlc_reference_caffenet.caffemodel";
  string mean_binaryproto = model_dir + "imagenet_mean.binaryproto";
  string labels_file = model_dir + "synset_words.txt";
  return BuildFromFiles(model_prototxt, pretrained_caffemodel, mean_binaryproto,
                        labels_file, recognizer, error);
}

bool ImageRecognizer::BuildFromFiles(const std::string& model_prototxt,
                                     const std::string& pretrained_caffemodel,
                                     const std::string& mean_binaryproto,
                                     const std::string& labels_file,
                                     ImageRecognizer* recognizer,
                                     std::string* error) {
  // Read network
  shared_ptr<caffe::Net<float> > net(
      new caffe::Net<float>(model_prototxt, caffe::TEST));
  net->CopyTrainedLayersFrom(pretrained_caffemodel);
  caffe::Blob<float>* input_layer = net->input_blobs()[0];
  int num_channels = input_layer->channels();
  if (!(num_channels == 3 || num_channels == 1)) {
    if (error != NULL) {
      *error = "Input layer should have 1 or 3 channels.";
    }
    return false;
  }

  // Get input geometry
  cv::Size input_geometry =
      cv::Size(input_layer->width(), input_layer->height());

  // Read mean
  cv::Mat mean =
      ReadMeanFile(mean_binaryproto, input_geometry, num_channels, error);
  if (error != NULL && *error != "") {
    return false;
  }

  // Read labels
  vector<string> labels;
  std::ifstream labels_in(labels_file.c_str());
  if (!labels_in) {
    if (error != NULL) {
      *error = "Unable to open labels file " + labels_file;
    }
    return false;
  }
  string line;
  while (std::getline(labels_in, line)) {
    labels.push_back(string(line));
  }
  caffe::Blob<float>* output_layer = net->output_blobs()[0];
  if (static_cast<int>(labels.size()) != output_layer->channels()) {
    if (error != NULL) {
      *error = "Number of labels is different from the output layer dimension.";
    }
    return false;
  }

  ImageRecognizer image_recognizer(net, mean, input_geometry, num_channels,
                                   labels);
  *recognizer = image_recognizer;
  return true;
}

cv::Mat ReadMeanFile(const string& mean_file, const cv::Size& input_geometry,
                     int num_channels, string* error) {
  caffe::BlobProto blob_proto;
  caffe::ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

  /* Convert from BlobProto to Blob<float> */
  caffe::Blob<float> mean_blob;
  mean_blob.FromProto(blob_proto);
  if (mean_blob.channels() != num_channels) {
    if (error != NULL) {
      *error = "Number of channels of mean file doesn't match input layer.";
    }
  }

  /* The format of the mean file is planar 32-bit float BGR or grayscale. */
  vector<cv::Mat> channels;
  float* data = mean_blob.mutable_cpu_data();
  for (int i = 0; i < num_channels; ++i) {
    /* Extract an individual channel. */
    cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
    channels.push_back(channel);
    data += mean_blob.height() * mean_blob.width();
  }

  /* Merge the separate channels into a single image. */
  cv::Mat mean;
  cv::merge(channels, mean);

  /* Compute the global mean pixel value and create a mean image
   * filled with this value. */
  cv::Scalar channel_mean = cv::mean(mean);
  mean = cv::Mat(input_geometry, mean.type(), channel_mean);
  return mean;
}

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
  return lhs.first > rhs.first;
}

static std::vector<int> Argmax(const std::vector<float>& v, int N) {
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

  std::vector<int> result;
  for (int i = 0; i < N; ++i) result.push_back(pairs[i].second);
  return result;
}
}  // namespace perception
}  // namespace rapid
