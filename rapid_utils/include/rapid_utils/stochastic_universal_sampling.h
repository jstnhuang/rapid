#ifndef _RAPID_UTILS_STOCHASTIC_UNIVERSAL_SAMPLING_H_
#define _RAPID_UTILS_STOCHASTIC_UNIVERSAL_SAMPLING_H_

#include <vector>

#include "rapid_utils/random.h"

namespace rapid {
namespace utils {
// Sample num_samples times from the given CDF. The sampled indices are
// returned in indices.
// The values of cdf must be between 0 and 1, and be in increasing order. The
// last number of cdf must be 1.
void StochasticUniversalSampling(const std::vector<double>& cdf,
                                 int num_samples, std::vector<int>* indices);

void StochasticUniversalSampling(const std::vector<double>& cdf,
                                 int num_samples,
                                 const RandomNumberInterface& rng,
                                 std::vector<int>* indices);
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_STOCHASTIC_UNIVERSAL_SAMPLING_H_
