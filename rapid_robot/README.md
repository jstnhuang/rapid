# rapid_robot
Contains libraries for common functionality in ROS robots.
Functionality that would be the same between the PR2, Fetch, etc. probably belong here.

## Executables
`joint_state_reader` gets joint positions for all joints or a subset of joints
~~~
rosrun rapid_robot joint_state_reader
rosrun rapid_robot joint_state_reader torso_lift_link head_pan_joint head_tilt_joint
~~~

## Libraries
- joint_state_reader.h: A joint state reader.

### Point cloud cameras
rapid::PointCloudCamera and rapid::RecordedPointCloudCamera implement the rapid::PointCloudCameraInterface.
This specifies a common interface for getting point clouds either from a real topic or from a pre-recorded point cloud.
The pre-recorded version is designed to work with rapid_perception's save_cloud executable.

~~~cpp
#include "rapid_robot/point_cloud_camera.h"
#include "rapid_utils/pcl_typedefs.h"

rapid::PointCloudCamera pr2_camera("/head_mount_kinect/depth_registered/points", "base_link");
PointCloudC::Ptr cloud = pr2_camera.cloud();

rapid::RecordedPointCloudCamera sim_camera;
sim_camera.LoadBag("/home/robot/data/scene1.bag");
PointCloudC::Ptr cloud = pr2_camera.cloud();
~~~
