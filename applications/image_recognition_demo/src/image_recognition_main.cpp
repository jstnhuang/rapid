#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/core/core.hpp"
#include "rapid_perception/image_recognition.h"

using rapid::perception::ImageRecognizer;

void GetPredictions(const std::string& image_file,
                    std::vector<std::pair<std::string, float> >* predictions);

void GetPredictions(ImageRecognizer& recognizer, const std::string& image_file,
                    std::vector<std::pair<std::string, float> >* predictions,
                    std::string* error) {
  cv::Mat image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  recognizer.set_image(image);
  std::vector<std::pair<std::string, float> > preds =
      recognizer.predictions(5, error);
  *predictions = preds;
}

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cout << "Usage: rosrun image_recognition image_recognition_main "
                 "cat1.jpg cat2.jpg dog1.jpg conv3" << std::endl;
    std::cout << "This demo predicts what's in the three images, and compares "
                 "how close the first image is to the other two in cosine "
                 "distance." << std::endl;
    return 0;
  }
  std::string model_dir = argv[1];
  std::string image1_file = argv[2];
  std::string image2_file = argv[3];
  std::string image3_file = argv[4];
  std::string layer_name = argv[5];

  ImageRecognizer recognizer;
  std::string error;
  ImageRecognizer::AlexNet(argv[1], &recognizer, &error);
  if (error != "") {
    std::cerr << "Error loading the AlexNet model from folder " << argv[1]
              << ": " << error << std::endl;
    return 1;
  }

  std::vector<std::pair<std::string, float> > predictions;
  GetPredictions(recognizer, image1_file, &predictions, &error);
  if (error != "") {
    std::cerr << "Error getting predictions: " << error << std::endl;
    return 1;
  }
  std::cout << image1_file << ": " << std::endl;
  for (size_t i = 0; i < predictions.size(); ++i) {
    std::cout << "  " << predictions[i].first << "\t" << predictions[i].second
              << std::endl;
  }
  cv::Mat layer1 = recognizer.layer(layer_name, &error);

  predictions.clear();
  GetPredictions(recognizer, image2_file, &predictions, &error);
  if (error != "") {
    std::cerr << "Error getting predictions: " << error << std::endl;
    return 1;
  }
  std::cout << image2_file << ": " << std::endl;
  for (size_t i = 0; i < predictions.size(); ++i) {
    std::cout << "  " << predictions[i].first << "\t" << predictions[i].second
              << std::endl;
  }
  cv::Mat layer2 = recognizer.layer(layer_name, &error);

  predictions.clear();
  GetPredictions(recognizer, image3_file, &predictions, &error);
  if (error != "") {
    std::cerr << "Error getting predictions: " << error << std::endl;
    return 1;
  }
  std::cout << image3_file << ": " << std::endl;
  for (size_t i = 0; i < predictions.size(); ++i) {
    std::cout << "  " << predictions[i].first << "\t" << predictions[i].second
              << std::endl;
  }
  cv::Mat layer3 = recognizer.layer(layer_name, &error);

  float distance_1_2 =
      layer1.dot(layer2) / (cv::norm(layer1) * cv::norm(layer2));
  float distance_1_3 =
      layer1.dot(layer3) / (cv::norm(layer1) * cv::norm(layer3));
  std::cout << layer_name << " layer size is " << layer1.size[0] << std::endl;
  std::cout << "Cosine distance between " << image1_file << " and "
            << image2_file << ": " << distance_1_2 << std::endl;
  std::cout << "Cosine distance between " << image1_file << " and "
            << image3_file << ": " << distance_1_3 << std::endl;

  return 0;
}
