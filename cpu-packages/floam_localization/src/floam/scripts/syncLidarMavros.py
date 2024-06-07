# synchronize Lidar with /mavros/local_position/pose topic to get the yaw
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import tf

syncedMavrosPublisher = rospy.Publisher("/synced/mavros/local_position/pose", PoseStamped, queue_size=1)
syncedVelodynePublisher = rospy.Publisher("/synced/velodyne_points", PointCloud2, queue_size=1)

mavrosData = []

def syncCallback(mavrosMsg, lidarMsg):
    global mavrosData
    syncedMavrosPublisher.publish(mavrosMsg)
    syncedVelodynePublisher.publish(lidarMsg)

    # convert the orientation from quaternion to euler angles using ros tf
    quaterions = [mavrosMsg.pose.orientation.x, mavrosMsg.pose.orientation.y, mavrosMsg.pose.orientation.z, mavrosMsg.pose.orientation.w]

    euler      = tf.transformations.euler_from_quaternion(quaterions)

    mavrosData.append(euler)

    np.save("mavrosYaw_demo.npy", np.array(mavrosData))

    print("Published synced data")

def syncMavrosLidarCb(mavrosTopic, lidarTopic):
    mavrosInfo = message_filters.Subscriber(mavrosTopic, PoseStamped)
    lidarInfo  = message_filters.Subscriber(lidarTopic,  PointCloud2)
    ts        = message_filters.ApproximateTimeSynchronizer([mavrosInfo, lidarInfo], 1, 1)
    ts.registerCallback(syncCallback)
    rospy.spin()

if __name__=="__main__":
    print("Synchronizing GPS and LIDAR data ...")
    rospy.init_node("sync")
    mavrosTopic = '/mavros/local_position/pose'
    lidarTopic = '/velodyne_points'
    syncMavrosLidarCb(mavrosTopic, lidarTopic)