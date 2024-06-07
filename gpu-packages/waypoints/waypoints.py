from utils import *
import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
import numpy as np


class Waypoints:
    def __init__(self) -> None:
        self.control_points = []
        self.tangent_points = []
        self.samples_per_bezier = 15
        self.bezier_points = []
        self.yaw_angles = []

        rospy.init_node('waypoints_node', anonymous=True)
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.beizier_points_publisher = rospy.Publisher('/trajectory', MarkerArray, queue_size=10)
        self.control_publisher = rospy.Publisher('/waypoints', MarkerArray, queue_size=10)


    def callback(self, data):
        rospy.loginfo("Clicked Point: [%f, %f, %f]", data.point.x, data.point.y, data.point.z)
        self.control_points.append([data.point.x, data.point.y, data.point.z])
        add_tangent_point(self.control_points, self.tangent_points)
        self.bezier_points, self.yaw_angles = calculate_piecewise_cubic_bezier_with_yaw(self.control_points, self.tangent_points, self.samples_per_bezier)  
        self.publish_points()

    def publish_points(self):
        control_marker_array = MarkerArray()
        control_marker_array.markers = []

        for i, points in enumerate(self.control_points):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = len(control_marker_array.markers)
            marker.pose.position.x = points[0]
            marker.pose.position.y = points[1]
            marker.pose.position.z = points[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            control_marker_array.markers.append(marker)

        if len((self.bezier_points)) > 4:
            bezier_marker_array = MarkerArray()
            for i, (point, yaw) in enumerate(zip(self.bezier_points, self.yaw_angles)):
                r = R.from_euler('xyz', [0, 0, yaw], degrees=False)
                r_quat = r.as_quat()

                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = "odom"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.id = len(bezier_marker_array.markers)
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                marker.pose.orientation.x = r_quat[0]
                marker.pose.orientation.y = r_quat[1]
                marker.pose.orientation.z = r_quat[2]
                marker.pose.orientation.w = r_quat[3]
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                bezier_marker_array.markers.append(marker)

            self.beizier_points_publisher.publish(bezier_marker_array)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_points()
            rospy.spin()

if __name__ == '__main__':
    try:
        waypoints = Waypoints()
        waypoints.run()
    except rospy.ROSInterruptException:
        pass
