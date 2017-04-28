string DEFAULT_POSITION = default position # Position controller, avoids obstacles and moves at a fixed speed
string OBSTACLE_SLOWDOWN = obstacle slowdown # Moves slower when close to obstacles, faster otherwise. Allows contact with obstacles.
string type
float64 max_speed # Maximum speed of the end-effector in the default controller
float64 max_force # Stops the arm if detected contact force exceeds this threshold
moveit_msgs/OrientationConstraint orientation_constraint
