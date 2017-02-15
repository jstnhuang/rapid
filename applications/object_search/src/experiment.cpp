#include "object_search/experiment.h"

namespace object_search {
ConfusionMatrix::ConfusionMatrix() : tp(0), tn(0), fp(0), fn(0) {}

double ConfusionMatrix::Precision() const {
  return tp / static_cast<double>(tp + fp);
}

double ConfusionMatrix::Recall() const {
  return tp / static_cast<double>(tp + fn);
}

double ConfusionMatrix::F1() const {
  double precision = Precision();
  double recall = Recall();
  return 2 * precision * recall / static_cast<double>(precision + recall);
}

void ConfusionMatrix::Merge(const ConfusionMatrix& other) {
  tp += other.tp;
  tn += other.tn;
  fp += other.fp;
  fn += other.fn;
}
}  // namespace object_search
