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
