# Type of the Action
## The first set of actions are actuations that move some part of the robot.
string ACTUATE_GRIPPER = actuate gripper # Open or close one of the grippers
string MOVE_TO_JOINT_GOAL = move to joint goal # Move the head, torso, or arm
string MOVE_TO_CARTESIAN_GOAL = move to cartesian goal # Move the arm
## The second set of actions are perception actions that update the world representation.
string DETECT_TABLETOP_OBJECTS = detect tabletop objects # Find tabletop objects
string FIND_CUSTOM_LANDMARK = find custom landmark # Find a custom landmark

string type

# If this is an actuation, this specifies what is being moved.
string GRIPPER = gripper # Used for robots that only have one gripper
string LEFT_GRIPPER = left gripper
string RIGHT_GRIPPER = right gripper
string HEAD = head
string TORSO = torso
string ARM = arm
string LEFT_ARM = left arm
string RIGHT_ARM = right arm
string JOINT_SET = joint set # A custom set of joints
string actuator_group

# Acutation specifications
## If type == ACTUATE_GRIPPER
control_msgs/GripperCommand gripper_command 

## If type == MOVE_TO_JOINT_GOAL
## Used to move the head, torso, arm, or a custom set of joints
trajectory_msgs/JointTrajectory joint_trajectory

## If type == MOVE_TO_CARTESIAN_GOAL
## This is used for pointing the head or moving the end-effector
geometry_msgs/Pose pose
Landmark landmark # A landmark that acts as the parent frame of the pose

## Specifies the kind of control to use for actuating the arm
ControlStrategy control_strategy

# Perception specifications
## If type == FIND_CUSTOM_LANDMARK
string custom_landmark_name
