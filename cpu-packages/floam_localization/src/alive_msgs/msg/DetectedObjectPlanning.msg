std_msgs/Header                 header
uint32                          id
string                          label
int32                           class_num
float32                         score   #Score as defined by the detection, Optional

#rectangle on groundplane
geometry_msgs/Point           top_left
geometry_msgs/Point           top_right
geometry_msgs/Point           bottom_left
geometry_msgs/Point           bottom_right
float32                         object_distance
float32                         object_velocity

