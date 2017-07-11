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

## Installing
- Copy the `rapid_pbd` and `rapid_pbd_msgs` folder to your catkin workspace.
- Clone `moveit_goal_builder`: `git clone git@github.com:jstnhuang/moveit_goal_builder.git`
- Get other dependencies through rosdep: `cd ~/catkin_ws; rosdep install --from-paths src --ignore-src --rosdistro=indigo -y`

## Running the system (PR2)
- Start up the robot (for simulation, run `roslaunch rapid_pbd pr2_sim.launch`)
  - [Kinect](https://github.com/hcrlab/wiki/blob/master/kinect/launch.md)
  - Websocket server: `roslaunch rosbridge_server rosbridge_websocket.launch`
  - TF republisher: `rosrun tf2_web_republisher tf2_web_republisher`
- Start the program executor: `roslaunch rapid_pbd pr2.launch sim:=false`
- Start the editor server: `roslaunch rapid_pbd editor.launch`
- Run the frontend: `cd rapid_pbd/frontend; polymer serve`
