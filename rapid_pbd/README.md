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
- Clone `transform_graph` (until updated version reaches debs): `git clone git@github.com:jstnhuang/transform_graph.git`
- Clone `surface_perception` (until updated version reaches debs): `git clone git@github.com:jstnhuang/surface_perception.git`
- To avoid mongodb errors, please use my fork of mongodb_store: https://github.com/jstnhuang/mongodb_store
- For Fetch development, install the fetch_ikfast_plugin package: 
- Get other dependencies through rosdep: `cd ~/catkin_ws; rosdep install --from-paths src --ignore-src --rosdistro=indigo -y`

## Running the system (PR2 simulation)
**Simulation**
- [ ] `roscore`
- [ ] Gazebo: `roslaunch pr2_gazebo pr2_empty_world.launch`
- [ ] Websocket and TF republisher: `roslaunch rapid_pbd web_prereqs.launch`
- [ ] MoveIt: `roslaunch pr2_moveit_config move_group.launch`
- [ ] Serve `pr2_description` to `localhost:8001`: `cd pr2_description; caddy`
- [ ] Publish a point cloud to `/mock_point_cloud`: `rosrun applications publish_saved_cloud ~/data/objects.bag`

**Rapid PbD**
- [ ] Start the program executor: `roslaunch rapid_pbd pr2.launch sim:=false`
- [ ] Start the editor server: `roslaunch rapid_pbd editor.launch`
- [ ] Run the frontend: `cd rapid_pbd/frontend; polymer serve`
