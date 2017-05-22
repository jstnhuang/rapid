Action[] actions # A list of actions to run concurrently.

string scene_id # An ID that can be used to look up a point cloud of the scene at this step.

# All of the landmarks found once this step executes.
#
# When a step is created or edited, this is filled in as perception actions get added
# to the step or existing perception actions are updated.
# Subsequent steps that are created or edited will search backwards to find needed
# landmarks.
#
# When a step is executed, the perception actions are re-run. The program executor
# maintains the world state (aka list of landmarks) in memory.
rapid_pbd_msgs/Landmark[] landmarks # All of the landmarks known at this step.

# The joint state that results once this step executes.
# This is used only for visualization purposes, not for specifying the program.
# It allows the interface to render a preview of the robot state for each step.
sensor_msgs/JointState end_joint_state
