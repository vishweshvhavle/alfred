# Publish the waypoints given to planner

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def publishPoints(xPts, yPts):
    markerPublisher = rospy.Publisher("/sampled_waypoints", MarkerArray, queue_size=100)
    count = 0
    markerArray = MarkerArray()
    r = rospy.Rate(20)
    
    for i in range(len(xPts)):
        marker      = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp    = rospy.Time.now()
        marker.type            = marker.SPHERE
        marker.action          = marker.ADD
        marker.scale.x         = 0.7
        marker.scale.y         = 0.7
        marker.scale.z         = 0.7
        marker.color.a         = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = xPts[i]
        marker.pose.position.y = yPts[i]
        marker.pose.position.z = 0
        marker.id = count
        marker.ns = str(count)
        markerArray.markers.append(marker)
        count += 1
    
    while not rospy.is_shutdown:
        markerPublisher.publish(markerArray)
        print("Marker array published")
        # TODO sleep appropriately
        r.sleep()
    

if __name__=="__main__":
    rospy.init_node("Sampled_waypoints_publisher")

    pathToWaypointsX = '/home/nuc2/alive/waypoints/carla/wayptsX_18Nov_1.txt'
    pathToWaypointsy = '/home/nuc2/alive/waypoints/carla/wayptsY_18Nov_1.txt'    

    # loop through the files and get the coordinates
    xPts = []
    yPts = []

    xFile = open(pathToWaypointsX,"r")
    yFile = open(pathToWaypointsy, "r")

    xLines = xFile.read().splitlines()
    yLines = yFile.read().splitlines()
    
    for x in xLines:
        xPts.append(float(x))
    
    for y in yLines:
        yPts.append(float(y))

    # now publish these points
    publishPoints(xPts, yPts)