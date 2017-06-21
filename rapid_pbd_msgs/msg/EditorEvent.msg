# Specifies an action taken by the PbD editor.
string CREATE = create program
string VIEW = view step
string UPDATE = update program
string DELETE = delete program
string DELETE_STEP = delete step
string DETECT_SURFACE_OBJECTS = detect surface objects
string type

# If type is CREATE, only program_info.name is needed.
# If type is VIEW, then program_info.db_id and step_num are needed.
# If type is UPDATE, all fields of program_info and program are needed.
# If type is DELETE, only program_info.db_id is needed.
# If type is DELETE_STEP, program_info.db_id and step_num are needed.
# If type is DETECT_SURFACE_OBJECTS, program_info.db_id and step_num are needed.
rapid_pbd_msgs/ProgramInfo program_info
rapid_pbd_msgs/Program program
int32 step_num
