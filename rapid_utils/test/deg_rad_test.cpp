#include "rapid_utils/deg_rad.h"

#include <gtest/gtest.h>
#include <math.h>

namespace rapid {
TEST(DegRadTest, ValueTest) {
  Degrees degrees(-30);
  EXPECT_FLOAT_EQ(-30, degrees.value());
  Radians radians(-0.5);
  EXPECT_FLOAT_EQ(-0.5, radians.value());
}

TEST(DegRadTest, DegToRad) {
  Degrees degrees(-180);
  Radians radians(degrees);
  EXPECT_FLOAT_EQ(-M_PI, radians.value());

  Degrees degrees2(0);
  Radians radians2(degrees2);
  EXPECT_FLOAT_EQ(0, radians2.value());
}
}  // namespace rapid

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
