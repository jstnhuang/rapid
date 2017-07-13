# Type of the landmark
# A TF frame ID
string TF_FRAME = tf frame
# A box from the tabletop object detector
string SURFACE_BOX = surface box
# A custom landmark
string CUSTOM_LANDMARK = custom landmark
# An AR tag or tag bundle
string AR_TAG = AR tag
string type

# Specifies the pose of the landmark.
# All landmarks and end-effector poses should be specified in the base frame so that
# an end-effector pose can always be transformed into the frame of a different landmark.
# This field can be blank if the landmark's pose is unknown at the time.
geometry_msgs/PoseStamped pose_stamped

# If type == TF_FRAME, name is the frame ID
# If type == SURFACE_BOX, name is an arbitrary name like "Object 1"
# If type == CUSTOM_LANDMARK, name is the name of the custom landmark
string name
geometry_msgs/Vector3 surface_box_dims # Dimensions of a tabletop box
uint32 ar_tag_id # The AR tag ID or master tag ID
