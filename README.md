# rapid
A ROS robotics library.

[![Build Status](https://travis-ci.org/jstnhuang/rapid.svg?branch=master)](https://travis-ci.org/jstnhuang/rapid)

This is especially designed for ROS Hydro and the PR2 robot.
It is still in active development.

## Summary
- **rapid_display**:
  An interface for showing interactive content on a touch screen.
  Includes a preliminary implementation based on [blinky](https://github.com/hcrlab/blinky).
- **rapid_main**:
  Demo programs that exercise the capabilities of blinky.
  It is a leaf package, it has no libraries and no one should depend on it.
- **rapid_manipulation**:
  Code related to arm manipulation and grasping.
- **rapid_msgs**:
  Message, service, and action files used in this library.
- **rapid_perception**:
  Code related to 2D and 3D computer vision.
- **rapid_pr2**:
  A PR2 class, which provides a simple API for the PR2's functionality.
- **rapid_ros**:
  Wrappers around common ROS functionality, including mock objects.
- **rapid_utils**:
  Simple utilities supporting the library.
- **rapid_viz**:
  Visualization tools, including a (hopefully) RAII Marker class.
