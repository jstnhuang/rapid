# Specifies the state of the robot arm controllers (frozen, relaxed, or perhaps others in the future).

# The arm is being held in place.
string FROZEN = frozen
# The arm is relaxed, suitable for kinesthetic demonstration.
string RELAXED = relaxed

# For single-arm robots, only the arm_controller field is used.
string arm_controller

# For dual-arm robots, the l_arm_controller and r_arm_controller fields are used.
string l_arm_controller
string r_arm_controller
