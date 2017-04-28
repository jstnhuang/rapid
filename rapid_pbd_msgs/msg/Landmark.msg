# Type of the landmark
string TF_FRAME = tf frame # A TF frame ID
string TABLETOP_BOX = tabletop box # A box from the tabletop object detector
string CUSTOM_LANDMARK = custom landmark # A custom landmark
string AR_TAG = AR tag # An AR tag or tag bundle
string POSE_STAMPED = pose stamped # An arbitrary pose
string type

string frame_id # If type == TF_FRAME
geometry_msgs/Vector3 tabletop_box_dims # Dimensions of a tabletop box
string custom_landmark_name # The name of the custom landmark
uint32 ar_tag_id # The AR tag ID or master tag ID
geometry_msgs/PoseStamped pose_stamped
