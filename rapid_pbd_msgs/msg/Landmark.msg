# Type of the landmark
string TF_FRAME = tf frame # A TF frame ID
string SURFACE_BOX = surface box # A box from the tabletop object detector
string CUSTOM_LANDMARK = custom landmark # A custom landmark
string AR_TAG = AR tag # An AR tag or tag bundle
string type

# Specifies the pose of the landmark.
# All landmarks and end-effector poses should be specified in the base frame so that
# an end-effector pose can always be transformed into the frame of a different landmark.
# This field can be blank if the landmark's pose is unknown at the time.
geometry_msgs/PoseStamped pose_stamped

string frame_id # If type == TF_FRAME
geometry_msgs/Vector3 surface_box_dims # Dimensions of a tabletop box
string custom_landmark_name # The name of the custom landmark
uint32 ar_tag_id # The AR tag ID or master tag ID
