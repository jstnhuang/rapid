# rapid_pr2

High-level API for the PR2.

## Executables

### gripper
Opens or closes the grippers.

~~~
rosrun rapid_pr2 gripper open left
rosrun rapid_pr2 gripper close right
rosrun rapid_pr2 gripper close right 50
~~~

### torso
Moves the torso up or down.

~~~
rosrun rapid_pr2 torso 0.3
~~~

## Libraries
- arm_joints.h: Lists the arm joints
- gripper.h: Controls the PR2's grippers
- torso.h: Controls the PR2's torso
