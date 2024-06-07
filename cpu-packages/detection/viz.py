import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Display image
    cv2.imshow("Detected Frame", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('frame_viewer', anonymous=True)
    rospy.Subscriber("/detection", Image, callback)

    # Keep python from exiting until this node is stopped
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
