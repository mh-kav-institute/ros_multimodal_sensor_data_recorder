# Outsight Augmented Cloud

# Time of sensor data acquisition.
Header header

# Number of points and layers.
uint32 number_of_points
uint32 number_of_layers

# Point reflectivities of all points (number_of_points).
uint8[] reflectivities

# Cartesian coordinates of all points (3 * number_of_points).
float32[] cartesian_coordinates

# List of tracked object IDs.
uint32[] object_ids