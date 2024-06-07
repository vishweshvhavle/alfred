# LaneChangeManeuver.msg
#
# Message defining a Lane Change Maneuver for use in planning. 
# This message should only be used to describe lane changes to adjacent lanes
# Lane changes that cross multiple lanes should be described using a sequence of maneuvers

# Approximate starting state of the maneuver
float64 start_dist # downtrack distance in m
float64 start_speed # vehicle speed in m/s
time start_time

# Approximate ending state of the maneuver
float64 end_dist 
float64 end_speed
time end_time

# ID of the starting lane of this maneuver
int64 starting_lane_id
# ID of the ending lane of this maneuver
int64 ending_lane_id