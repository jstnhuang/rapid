# A Task, used in the CustomLandmarks experiment, represents a scene and a set of landmarks to be detected.
# The labels are saved in a separate collection.
string name # Usually the same as the scene name, but the same scene can be used for multiple tasks
string scene_name
object_search_msgs/Label[] labels
