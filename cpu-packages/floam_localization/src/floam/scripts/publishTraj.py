# read the numpy file and publish the trajectory

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

floamOrigPath = "/home/deepak/IIITD/catkin_ws/src/floam/scripts/test_12Nov_bag_football_track.npy"

pathFloam = Path()
pathMapper  = Path()

rospy.init_node("traj_comparison_floam_mapper")
floamPub = rospy.Publisher("/floam__orig_traj", Path, queue_size=1)


floamOrig = np.load(floamOrigPath)

for i in range(floamOrig.shape[0]):

    pFloam = PoseStamped()

    pFloam.pose.position.x = floamOrig[i,1]
    pFloam.pose.position.y = floamOrig[i,2]
    pFloam.pose.position.z = floamOrig[i,3]
    pFloam.pose.orientation.w = 1.0


    pathFloam.poses.append(pFloam)

    pathFloam.header.frame_id = "/map"

    pathFloam.header.stamp = rospy.Time.now()


while not rospy.is_shutdown():
    print("Publishing the trajectory")
    floamPub.publish(pathFloam)
