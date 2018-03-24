#ifndef _RAPID_UTILS_STOCHASTIC_UNIVERSAL_SAMPLING_H_
#define _RAPID_UTILS_STOCHASTIC_UNIVERSAL_SAMPLING_H_

#include <vector>

#include "rapid_utils/random.h"

namespace rapid {
namespace utils {
/// \brief Sample randomly from a CDF with a uniform sampling method.
///
/// \param[in] cdf The CDF. All values must be between 0 and 1 in ascending
///   order. The last number must be 1.
/// \param[in] num_samples The number of samples to draw.
/// \param[out] indices The indices sampled from the CDF.
void StochasticUniversalSampling(const std::vector<double>& cdf,
                                 int num_samples, std::vector<int>* indices);

/// \brief Sample randomly from a CDF with a uniform sampling method.
///
/// \param[in] cdf The CDF. All values must be between 0 and 1 in ascending
///   order. The last number must be 1.
/// \param[in] num_samples The number of samples to draw.
/// \param[in] rng A custom random number generator. This is also used during
///   testing to test with a mock RNG.
/// \param[out] indices The indices sampled from the CDF.
void StochasticUniversalSampling(const std::vector<double>& cdf,
                                 int num_samples,
                                 const RandomNumberInterface& rng,
                                 std::vector<int>* indices);
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_STOCHASTIC_UNIVERSAL_SAMPLING_H_
