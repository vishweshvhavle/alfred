std_msgs/Header                 header

uint32                          id
string                          label
int32                           class_num
float32                         score   #Score as defined by the detection, Optional
#std_msgs/ColorRGBA              color   # Define this object specific color
#bool                            valid   # Defines if this object is valid, or invalid as defined by the filtering

################ 3D BB
#string                          space_frame #3D Space coordinate frame of the object, required if pose and dimensions are defines
#geometry_msgs/Pose              pose
#geometry_msgs/Vector3           dimensions
#geometry_msgs/Vector3           variance
#geometry_msgs/Twist             velocity
#geometry_msgs/Twist             acceleration

#sensor_msgs/PointCloud2         pointcloud

#geometry_msgs/PolygonStamped    convex_hull
#autoware_msgs/LaneArray         candidate_trajectories

#bool                            pose_reliable
#bool                            velocity_reliable
#bool                            acceleration_reliable

############### 2D Rect
#string                          image_frame # Image coordinate Frame,        Required if x,y,w,h defined
int32                           xmin           # X coord in image space(pixel) of the initial point of the Rect
int32                           ymin           # Y coord in image space(pixel) of the initial point of the Rect
int32                           xmax       # Width of the Rect in pixels
int32                           ymax      # Height of the Rect in pixels
#float32                         angle       # Angle [0 to 2*PI), allow rotated rects
float32                         object_distance
float32                         object_velocity

#sensor_msgs/Image               roi_image

