###########################################
# For saving GPS-tagged LIDAR pointclouds #
###########################################

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, NavSatFix

syncedGpsPublisher        = rospy.Publisher('/synced_gps', NavSatFix, queue_size=1)
syncedPointCloudPublisher = rospy.Publisher('/synced_pointCloud', PointCloud2, queue_size=1)

def syncCallback(gpsMsg, lidarMsg):
    syncedGpsPublisher.publish(gpsMsg)
    syncedPointCloudPublisher.publish(lidarMsg)
    print('Published synced data')


def syncGpsLidar(gpsTopic, lidarTopic):
    gpsInfo   = message_filters.Subscriber(gpsTopic, NavSatFix)
    lidarInfo = message_filters.Subscriber(lidarTopic, PointCloud2)
    ts        = message_filters.ApproximateTimeSynchronizer([gpsInfo, lidarInfo], 1, 1)
    ts.registerCallback(syncCallback)
    rospy.spin()

if __name__=="__main__":
    print("Synchronizing GPS and LIDAR data ...")
    rospy.init_node("sync")
    gpsTopic = '/mavros/global_position/global'
    lidarTopic = '/velodyne_points'
    syncGpsLidar(gpsTopic, lidarTopic)
    