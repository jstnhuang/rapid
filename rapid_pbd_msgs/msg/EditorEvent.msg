# Specifies an action taken by the PbD editor.
string CREATE = create program
string OPEN = open program
string UPDATE = update program
string DELETE = delete program
string action

# If action is CREATE, only program_info.name is needed.
# If action is OPEN or DELETE, only program_info.db_id is needed.
# If action is UPDATE, all fields of program_info and program are needed.
rapid_pbd_msgs/ProgramInfo program_info
rapid_pbd_msgs/Program program
