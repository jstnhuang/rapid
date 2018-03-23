# rapid_collision
Simple APIs for collision checking in ROS

Oriented bounding box check:
```cpp
#include "rapid_collision/collision_checks.h"

geometry_msgs::Pose pose1 = ...;
geometry_msgs::Vector3 scale1 = ...;
geometry_msgs::Pose pose2 = ...;
geometry_msgs::Vector3 scale2 = ...;
bool is_collision = rapid::AreObbsInCollision(pose1, scale1, pose2, scale2));
```
