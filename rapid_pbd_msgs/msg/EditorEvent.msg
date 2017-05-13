# Specifies an action taken by the PbD editor.
string CREATE = create program
string OPEN = open program
string UPDATE = update program
string DELETE = delete program
string ADD_ACTION = add action
string type

# If type is CREATE, only program_info.name is needed.
# If type is OPEN or DELETE, only program_info.db_id is needed.
# If type is UPDATE, all fields of program_info and program are needed.
rapid_pbd_msgs/ProgramInfo program_info
rapid_pbd_msgs/Program program

# If type is ADD_ACTION, then step_index specifies which step the editor should add the action to.
# action partially specifies the action to add, the rest is then filled out by the server.
# For example, it may say to add a joint goal for the right arm.
# The editor frontend may also add actions by simply sending a message with type == UPDATE,
# which is useful for edits that don't require the robot's cooperation.
int32 step_index
rapid_pbd_msgs/Action action
