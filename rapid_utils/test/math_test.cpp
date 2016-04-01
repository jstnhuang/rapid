#include "rapid_utils/math.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using geometry_msgs::Point;
using geometry_msgs::Vector3;

namespace rapid {
namespace utils {
// Interval tests. The start and end points of the first interval are coded as
// either B, I, or O. B = before second interval, I = in second interval, and O
// = after second interval.
TEST(MathTest, IntervalsIntersect) {
  EXPECT_EQ(false, IntervalsIntersect(-1, 0, 1, 2));  // BB
  EXPECT_EQ(true, IntervalsIntersect(-1, 1, 0, 2));   // BI
  EXPECT_EQ(true, IntervalsIntersect(-1, 2, 0, 1));   // BO
  EXPECT_EQ(true, IntervalsIntersect(0, 1, -1, 2));   // II
  EXPECT_EQ(true, IntervalsIntersect(0, 2, -1, 1));   // IO
  EXPECT_EQ(false, IntervalsIntersect(1, 2, -1, 0));  // OO
}

TEST(MathTest, AabbXYIntersect) {
  Point pos1;
  pos1.x = 1;
  pos1.y = 1;
  Vector3 scale1;
  scale1.x = 2;
  scale1.y = 2;
  Vector3 scale2 = scale1;
  Point pos2;

  pos2.x = 1;  // Square 2 is above, not touching
  pos2.y = 4;
  EXPECT_EQ(false, AabbXYIntersect(pos1, scale1, pos2, scale2));

  pos2.x = 1;  // Square 2 is above, touching
  pos2.y = 3;
  EXPECT_EQ(true, AabbXYIntersect(pos1, scale1, pos2, scale2));

  pos2.x = 4;  // Square 2 is right, not touching
  pos2.y = 1;
  EXPECT_EQ(false, AabbXYIntersect(pos1, scale1, pos2, scale2));

  pos2.x = 3;  // Square 2 is right, touching
  pos2.y = 1;
  EXPECT_EQ(true, AabbXYIntersect(pos1, scale1, pos2, scale2));

  pos2.x = 4;  // Square 2 is above and to the right, not touching
  pos2.y = 4;
  EXPECT_EQ(false, AabbXYIntersect(pos1, scale1, pos2, scale2));

  pos2.x = 2;  // Square 2 is above and to the right, intersecting
  pos2.y = 2;
  EXPECT_EQ(true, AabbXYIntersect(pos1, scale1, pos2, scale2));
}
}  //  namespace utils
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
