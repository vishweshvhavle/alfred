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
pcPub = rospy.Publisher("/transformed_curb_points", PointCloud2, queue_size=100)

def pcCb(msg):
    # convert msg to numpy array
    # add current time in the header
    # rest everything remains the same
    print('here')
    global Rz, pcPub

    pcNp = ros_numpy.numpify(msg)

    pointsNp = np.zeros((pcNp.shape[0], 3))

    pointsNp[:,0] = pcNp['x']
    pointsNp[:,1] = pcNp['y']
    pointsNp[:,2] = pcNp['z']
    
    theta = 15.0*3.14/180.0

    R = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])

    print(pointsNp)
    # np.save("curbs.npy", pointsNp)

    pointsTf        = np.zeros(pointsNp.shape)
    
    pts             = pointsNp[:,0:2]
    
#    print(f'PtsT has shape: {pts.T.shape}')
#    print(f'R has shape: {R.shape}')
    pointsTf[:,0:2] = np.dot(R, pts.T).T
    pointsTf[:,2]   = pointsNp[:,2]

    # publish this as a pointcloud2 message
    ros_dtype = PointField.FLOAT32
    itemsize = np.dtype(np.float32).itemsize
    data = pointsTf.astype(np.float32)

    points = data
    msg = PointCloud2()

    msg.header.stamp = msg.header.stamp

    msg.header.frame_id = "map"

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
    #msg.is_dense = True
    
    msg.data = np.asarray(points, np.float32).tostring()
    
    pcPub.publish(msg)
    
    print("Message published")


if __name__=="__main__":
    rospy.init_node("transform_curb_pointCloud")
    rospy.Subscriber("/curb_points_full", PointCloud2, pcCb)
    rospy.spin()
