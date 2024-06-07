#! /usr/bin/env python

import rospy
import ros_numpy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import *

# final trans_mat on Feb 2, 2023, without lanelet translation
# [[-0.67216473  0.74040163  0.          0.        ]
#  [-0.74040163 -0.67216473  0.          0.        ]
#  [ 0.          0.          1.          0.        ]
#  [ 0.          0.          0.          1.        ]]

R_ll = np.load("../resources/testbed_gate4_rotation.npy")

# 15 degree rotation on the lidar map to make the lidar straight
theta = math.radians(15.0)
R_lidar = np.array([[math.cos(theta), -math.sin(theta)],
                [math.sin(theta), math.cos(theta)]])

# Combined transformation matrix
trans_mat = np.eye(4)
trans_mat[:2, :2] = R_ll.dot(R_lidar)   # lidar rotation first and then lanelet rotation

# TODO - uncomment if we really need translation
# trans_ll = np.load("../resources/testbed_gate4_origin_translation.npy") 
# trans_mat[:2, 3] = trans_ll[:2, 2]

pcPub = rospy.Publisher("/transformed_points", PointCloud2, queue_size=10)

# TODO - uncomment if need lanelet pose rotation
# posePub = rospy.Publisher("/transformed_pose", PoseStamped, queue_size=10)

# def poseCb(msg):
#     # Rotation done on Lidar Map to align with lanelet map (same as R_ll)
#     R = [[-0.8325578,  0.55393818, 0.0, 0.0],
#          [-0.55393818, -0.8325578, 0.0, 0.0 ],
#          [0.0, 0.0, 1.0, 0.0], 
#          [0.0, 0.0, 0.0, 1.0]]
#     q = quaternion_from_euler(0, 0, 0)
#     q_rot = quaternion_from_matrix(R)
#     q[0] = msg.pose.orientation.x
#     q[1] = msg.pose.orientation.y
#     q[2] = msg.pose.orientation.z
#     q[3] = msg.pose.orientation.w
#     q_new = quaternion_multiply(q_rot, q)
#     msg.header.frame_id = "map"
#     msg.pose.orientation.x = q_new[0]
#     msg.pose.orientation.y = q_new[1]
#     msg.pose.orientation.z = q_new[2]
#     msg.pose.orientation.w = q_new[3]
#     print(msg)
#     posePub.publish(msg)


def pcCb(pc_msg):
    pc_arr = ros_numpy.numpify(pc_msg)
    points = np.ones((pc_arr.shape[0], 4))
    points[:, 0] = pc_arr['x']
    points[:, 1] = pc_arr['y']
    points[:, 2] = pc_arr['z']
    
    # Rotation and translation to align with lanelet map
    points = np.dot(trans_mat, points.T).T

    out_pc_arr = np.zeros(len(points), dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.float32),
            ])
    out_pc_arr['x'] = points[:, 0]
    out_pc_arr['y'] = points[:, 1]
    out_pc_arr['z'] = points[:, 2]
    out_pc_arr['intensity'] = pc_arr['intensity']
    out_pc_arr['ring'] = pc_arr['ring']

    out_msg = ros_numpy.msgify(PointCloud2, out_pc_arr, stamp=pc_msg.header.stamp, frame_id='base_link')
    pcPub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("lanelet_rotation_correction")
    # rospy.Subscriber("/lidarPose", PoseStamped, poseCb)
    rospy.Subscriber("/lidar103/velodyne_points", PointCloud2, pcCb)
    print("publishing transformed_points")
    rospy.spin()