#include "rapid_utils/stochastic_universal_sampling.h"

#include <cstdlib>

namespace rapid {
namespace utils {

void StochasticUniversalSampling(const std::vector<double>& cdf,
                                 int num_samples, std::vector<int>* indices) {
  RandomNumber rng;
  StochasticUniversalSampling(cdf, num_samples, rng, indices);
}

void StochasticUniversalSampling(const std::vector<double>& cdf,
                                 int num_samples,
                                 const RandomNumberInterface& rng,
                                 std::vector<int>* indices) {
  double step = 1.0 / num_samples;
  double random = static_cast<double>(rng.rand()) / RAND_MAX;  // [0, 1]
  double threshold = step * random;

  for (int s = 0; s < num_samples; ++s) {
    int index = 0;
    while (threshold > cdf[index]) {
      ++index;
      if (index == static_cast<int>(cdf.size())) {
        index -= 1;
        break;
      }
    }
    indices->push_back(index);
    threshold += step;
  }
}
}  // namespace utils
}  // namespace rapid
