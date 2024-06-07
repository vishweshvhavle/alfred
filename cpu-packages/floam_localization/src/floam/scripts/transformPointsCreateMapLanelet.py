#! /usr/bin/env python
###############################################
# Compensate for extra rotation in pointcloud #
###############################################

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
import numpy as np
from std_msgs.msg import Header
import math

# convert from PointCloud2 to numpy array
# perform transformation on the point cloud -> rotate by -15 degrees in the pitch
pcPub = rospy.Publisher("/transformed_points", PointCloud2, queue_size=100)


def pcCb(msg):
    # convert msg to numpy array
    # add current time in the header
    # rest everything remains the same
    print('here')
    global Rz, pcPub

    pcNp = ros_numpy.numpify(msg)

    pointsNp = np.zeros((pcNp.shape[0], 3))

    pointsNp[:, 0] = pcNp['x']
    pointsNp[:, 1] = pcNp['y']
    pointsNp[:, 2] = pcNp['z']

    theta = 15.0*3.14/180.0

    R = np.array([[math.cos(theta), -math.sin(theta)],
                 [math.sin(theta), math.cos(theta)]])

    print(pointsNp)
    # np.save("curbs.npy", pointsNp)

    pointsTf = np.zeros(pointsNp.shape)
    pointsTfT = np.zeros(pointsNp.shape)
    pts = pointsNp[:, 0:2]
    pointsTf[:, 0:2] = np.dot(R, pts.T).T
    pts2 = pointsTf[:, 0:2]

    R_new = np.load("../resources/test_track_0.5rate.pcd_R.npy")
    trans = np.load("../resources/test_track_0.5rate.pcd_origin_trans.npy")
    print("Rnew", R_new.shape)
    
    pointsTf[:,0:2] = np.dot(R_new, pts2.T).T

    pointsTfT = np.dot(trans[:,0:2], pointsTf[:,0:2].T).T
    pointsTfT[:,2] = pointsNp[:,2]
    pointsTf[:,2] = pointsNp[:,2]

    data = pointsTfT.astype(np.float32)

    points = data
    msg = PointCloud2()

    msg.header.stamp = msg.header.stamp
    msg.header.frame_id = "base_link"
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]

    else:
        msg.height = 1
        msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]

    msg.is_bigendian = False

    msg.point_step = 12

    msg.row_step = msg.point_step * points.shape[0]

    msg.is_dense = int(np.isfinite(points).all())
    # msg.is_dense = True

    msg.data = np.asarray(points, np.float32).tostring()

    pcPub.publish(msg)

    print("Message published")


if __name__ == "__main__":
    rospy.init_node("transform_pointCloud")
    rospy.Subscriber("/lidar1/velodyne_points", PointCloud2, pcCb)
    rospy.spin()
