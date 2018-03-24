# rapid
[![Build Status](https://travis-ci.org/jstnhuang/rapid.svg?branch=indigo-devel)](https://travis-ci.org/jstnhuang/rapid)
[![Coverage Status](https://coveralls.io/repos/github/jstnhuang/rapid/badge.svg?branch=indigo-devel)](https://coveralls.io/github/jstnhuang/rapid?branch=indigo-devel)

A ROS robotics library.
- [Code API](https://jstnhuang.github.io/rapid/)

Currently, this supports [ROS Indigo](http://www.ros.org/reps/rep-0003.html#indigo-igloo-may-2014).

## Installation
~~~
cd ~/catkin_ws/src
git clone https://github.com/jstnhuang/rapid.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
catkin build
~~~

If you get a bunch of gmock errors, the issue is probably that the system version of gmock does not match the version of gmock that rapid expects.
To fix this issue, compile and install the latest version of [googletest](https://github.com/google/googletest).
~~~ {.cpp}
mkdir ~/local
cd ~/local
git clone git@github.com:google/googletest.git
cd googletest
mkdir mybuild
cd mybuild
cmake ..
make
sudo make install
~~~

See [.travis.yml](.travis.yml) for an example installation.

## Summary
- rapid_collision: Collision checking (will probably move into a util library at some point).
- rapid_db: A simple interface for saving ROS messages to MongoDB.
- rapid_fetch: Code for controlling the Fetch robot.
- rapid_manipulation: Code related to arm manipulation and grasping.
- rapid_msgs: Deprecated. Holds messages related to perception.
- rapid_perception: Code related to 2D and 3D computer vision.
- rapid_pr2: A PR2 class, which provides a simple API for the PR2's functionality.
- rapid_ros: Wrappers around common ROS functionality.
- rapid_testing: Utilities for unit testing / ROS testing.
- rapid_utils: Simple utilities supporting the library.
- rapid_viz: Visualization tools.
