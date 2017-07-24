# Rapid PbD

Rapid PbD is a programming by demonstration (PbD) system for the PR2 and Fetch robots.
The goal of the system is to provide an easy way to program manipulation actions that can be used in other applications.

## Program model
Users use the Rapid PbD interface to create *programs*.
A program is represented using the `rapid_pbd_msgs/Program` msg.
The system provides an actionlib interface for running programs.

A program consists of a sequence of *steps*, and each step consists of one or more *actions*.
There can be different types of actions, including moving the arm, moving the head, and detecting tabletop objects.
The actions of a step are run in parallel, but the steps run in sequence.
For example, in one step, you can point the head down and move the robot's arms to the side, and in the next step, you can detect tabletop objects.

## Getting started
- [Development setup](https://github.com/jstnhuang/rapid/wiki/Rapid-PbD-development-setup)
- [Running Rapid PbD](https://github.com/jstnhuang/rapid/wiki/Running-Rapid-PbD)
