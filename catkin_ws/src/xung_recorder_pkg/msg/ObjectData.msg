# Outsight custom message to define the tracked object data.

# ID of the tracked object.
uint32 id

# Pose of the tracked object.
geometry_msgs/Pose pose

# Size of the bounding box around the tracked object.
geometry_msgs/Vector3 box_size

# Current speed of tracked object.
geometry_msgs/Twist speed

# Class of the tracked object (as defined in OSEF library).
string object_class