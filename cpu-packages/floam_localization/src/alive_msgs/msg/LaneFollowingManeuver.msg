# LaneFollowingManeuver.msg
#
# Message defining a Lane Following Maneuver for use in planning. 

# Approximate starting state of the maneuver
float64 start_dist # downtrack distance in m
float64 start_speed # vehicle speed in m/s
time start_time

# Approximate ending state of the maneuver
float64 end_dist
float64 end_speed
time end_time

# List of lanes this maneuver will cover. They should all be contiguous lanes connected end to end (ie. no lane changes)
int64[] lane_ids
