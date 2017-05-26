# Specifies an action taken by the PbD editor.
string CREATE = create program
string VIEW = view step
string UPDATE = update program
string DELETE = delete program
string type

# If type is CREATE, only program_info.name is needed.
# If type is VIEW, then program_info.db_id and step_num are needed.
# If type is DELETE, only program_info.db_id is needed.
# If type is UPDATE, all fields of program_info and program are needed.
rapid_pbd_msgs/ProgramInfo program_info
rapid_pbd_msgs/Program program
int32 step_num
