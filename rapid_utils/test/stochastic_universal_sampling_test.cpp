#include "rapid_utils/stochastic_universal_sampling.h"

#include <vector>

#include "gtest/gtest.h"

#include "rapid_utils/random.h"

namespace rapid {
namespace utils {
TEST(SamplingTest, CommonCase) {
  std::vector<double> cdf;
  cdf.push_back(0.1);
  cdf.push_back(0.2);
  cdf.push_back(0.3);
  cdf.push_back(0.5);
  cdf.push_back(0.8);
  cdf.push_back(1);
  std::vector<int> indices;
  MockRandomNumber rng;
  // Make rand() return 0.05 * RAND_MAX
  rng.set_rand(static_cast<int>(0.5 + 0.05 * RAND_MAX));
  StochasticUniversalSampling(cdf, 4, rng, &indices);

  // threshold values will be 0.05, 0.3, 0.55, and 0.8
  // So CDF indices are 0, 2, 4, 4
  EXPECT_EQ(4, indices.size());
  EXPECT_EQ(0, indices[0]);
  EXPECT_EQ(2, indices[1]);
  EXPECT_EQ(4, indices[2]);
  EXPECT_EQ(4, indices[3]);
}

TEST(SamplingTest, RandMax) {
  std::vector<double> cdf;
  cdf.push_back(0.1);
  cdf.push_back(0.2);
  cdf.push_back(0.3);
  cdf.push_back(0.5);
  cdf.push_back(0.8);
  cdf.push_back(1);
  std::vector<int> indices;
  MockRandomNumber rng;
  // Make rand() return RAND_MAX
  rng.set_rand(RAND_MAX);
  StochasticUniversalSampling(cdf, 4, rng, &indices);

  // threshold values will be 0.25, 0.5, 0.75, and 1
  // So CDF indices are 2, 3, 4, 5
  EXPECT_EQ(4, indices.size());
  EXPECT_EQ(2, indices[0]);
  EXPECT_EQ(3, indices[1]);
  EXPECT_EQ(4, indices[2]);
  EXPECT_EQ(5, indices[3]);
}

TEST(SamplingTest, RandZero) {
  std::vector<double> cdf;
  cdf.push_back(0.1);
  cdf.push_back(0.2);
  cdf.push_back(0.3);
  cdf.push_back(0.5);
  cdf.push_back(0.8);
  cdf.push_back(1);
  std::vector<int> indices;
  MockRandomNumber rng;
  rng.set_rand(0);
  StochasticUniversalSampling(cdf, 4, rng, &indices);

  // threshold values will be 0, 0.25, 0.5, and 0.75
  // So CDF indices are 0, 2, 3, 4
  EXPECT_EQ(4, indices.size());
  EXPECT_EQ(0, indices[0]);
  EXPECT_EQ(2, indices[1]);
  EXPECT_EQ(3, indices[2]);
  EXPECT_EQ(4, indices[3]);
}
}  //  namespace utils
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
