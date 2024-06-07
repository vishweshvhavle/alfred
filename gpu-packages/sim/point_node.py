#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math

def add_vectors(v1, v2):
    return [a + b for a, b in zip(v1, v2)]

def scale_vector(v, scalar):
    return [a * scalar for a in v]

def magnitude(v):
    return math.sqrt(sum(a**2 for a in v))

def get_cubic_bezier(p0, p1, p2, p3, t):
    c0 = (1 - t) ** 3
    c1 = 3 * (1 - t) ** 2 * t
    c2 = 3 * (1 - t) * t ** 2
    c3 = t ** 3

    v0 = scale_vector(p0, c0)
    v1 = scale_vector(p1, c1)
    v2 = scale_vector(p2, c2)
    v3 = scale_vector(p3, c3)

    return add_vectors(add_vectors(v0, v1), add_vectors(v2, v3))

class PointClickSubscriber:
    def __init__(self):
        rospy.init_node('point_click_subscriber', anonymous=True)
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.point_publisher = rospy.Publisher('bezier_markers', MarkerArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.points_array = []
        self.marker_id = 0

    def callback(self, data):
        rospy.loginfo("Clicked Point: [%f, %f, %f]", data.point.x, data.point.y, data.point.z)
        self.points_array.append([data.point.x, data.point.y, data.point.z])
        self.publish_points()

    def calculate_tangent_points(self):
        num_points = len(self.points_array)
        if num_points < 3:
            return []

        tangent_points = []

        for i in range(1, num_points - 1):
            p0 = self.points_array[i - 1]
            p1 = self.points_array[i]
            p2 = self.points_array[i + 1]

            v01 = [(p1[0] - p0[0]), (p1[1] - p0[1]), (p1[2] - p0[2])]
            v12 = [(p2[0] - p1[0]), (p2[1] - p1[1]), (p2[2] - p1[2])]

            tangent_point1 = [p1[j] + v01[j] * 0.3 for j in range(3)]
            tangent_point2 = [p1[j] + v12[j] * 0.3 for j in range(3)]

            tangent_points.append([tangent_point1, tangent_point2])

        return tangent_points

    def publish_points(self):
        marker_array = MarkerArray()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # Create markers for control points
        for i in range(len(self.points_array)):
            marker = Marker()
            marker.header.frame_id = "base_link"  # Set the frame of reference for the points
            marker.id = self.marker_id
            self.marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.points_array[i][0]
            marker.pose.position.y = self.points_array[i][1]
            marker.pose.position.z = self.points_array[i][2]
            marker.scale.x = 0.1  # Set the size of the markers
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0  # Set the transparency of the markers
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        # Create markers for the cubic Bezier curve
        if len(self.points_array) >= 4:
            tangent_points = self.calculate_tangent_points()
            bezier_points = []
            for i in range(len(self.points_array) - 1):
                p0 = self.points_array[i]
                p3 = self.points_array[i + 1]
                if i == 0:
                    p1 = tangent_points[0][1]
                else:
                    p1 = tangent_points[i - 1][0]

                if i == len(self.points_array) - 2:
                    p2 = tangent_points[-1][0]
                else:
                    p2 = tangent_points[i][1]

                t = 0.0
                delta_t = 0.01  # Increase for smoother curve, decrease for performance
                while t <= 1.0:
                    bezier_point = get_cubic_bezier(p0, p1, p2, p3, t)
                    marker = Marker()
                    marker.header.frame_id = "base_link"
                    marker.id = self.marker_id
                    self.marker_id += 1
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = bezier_point[0]
                    marker.pose.position.y = bezier_point[1]
                    marker.pose.position.z = bezier_point[2]
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker_array.markers.append(marker)
                    t += delta_t

        self.point_publisher.publish(marker_array)

        self.rate.sleep()

if __name__ == '__main__':
    try:
        point_click_subscriber = PointClickSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
