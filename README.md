# rapid
A ROS robotics library.

[![Build Status](https://travis-ci.org/jstnhuang/rapid.svg?branch=indigo-devel)](https://travis-ci.org/jstnhuang/rapid)
[![Coverage Status](https://coveralls.io/repos/github/jstnhuang/rapid/badge.svg?branch=indigo-devel)](https://coveralls.io/github/jstnhuang/rapid?branch=indigo-devel)

This is especially designed for ROS Indigo and the PR2 robot.
It is still in active development.

## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/hcrlab/blinky.git
git clone https://github.com/jstnhuang/stf.git
git clone https://github.com/jstnhuang/mongo_msg_db_msgs.git
git clone https://github.com/jstnhuang/mongo_msg_db.git
git clone https://github.com/jstnhuang/rapid.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
catkin build
```

If you get a bunch of gmock errors, the issue is probably that the system version of gmock does not match the version of gmock that rapid expects.
To fix this issue, compile and install the latest version of [googletest](https://github.com/google/googletest).
```
mkdir ~/local
cd ~/local
git clone git@github.com:google/googletest.git
cd googletest
mkdir mybuild
cd mybuild
cmake ..
make
sudo make install
```

See [.travis.yml](.travis.yml) for an example installation.

## Summary
- **[rapid_display](rapid_display)**:
  An interface for showing interactive content on a touch screen.
  Includes a preliminary implementation based on [blinky](https://github.com/hcrlab/blinky).
- **[rapid_main](rapid_main)**:
  Demo programs that exercise the capabilities of blinky.
  It is a leaf package, it has no libraries and no one should depend on it.
- **[rapid_manipulation](rapid_manipulation)**:
  Code related to arm manipulation and grasping.
- **[rapid_msgs](rapid_msgs)**:
  Message, service, and action files used in this library.
- **[rapid_perception](rapid_perception)**:
  Code related to 2D and 3D computer vision.
- **[rapid_pr2](rapid_pr2)**:
  A PR2 class, which provides a simple API for the PR2's functionality.
- **[rapid_ros](rapid_ros)**:
  Wrappers around common ROS functionality, including mock objects.
- **[rapid_utils](rapid_utils)**:
  Simple utilities supporting the library.
- **[rapid_viz](rapid_viz)**:
  Visualization tools, including a (hopefully) RAII Marker class.
