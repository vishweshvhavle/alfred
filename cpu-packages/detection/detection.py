import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rgb_image_path = '/home/alfred/alfred_packages/detection/output/rgb_image_detected.png'

def capture_frame(pipeline):
    # Capture frames
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert frames to NumPy arrays
    rgb_image = np.asanyarray(color_frame.get_data())
    return rgb_image

def detect_and_plot(frame, model):
    # Run detection
    results = model.predict(frame, conf=0.65)
    predicted_frame = None
    
    # Visualize the results
    for r in results:
        # Plot results image
        predicted_frame = r.plot()  # BGR-order numpy array
        cv2.imwrite(rgb_image_path, predicted_frame)
    
    return predicted_frame

def main():
    rospy.init_node('frame_publisher', anonymous=True)
    pub = rospy.Publisher('/detection', Image, queue_size=10)
    bridge = CvBridge()

    # Set up the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Load a pretrained YOLOv8n model
    model = YOLO('yolov8n.pt')

    try:
        while not rospy.is_shutdown():
            frame = capture_frame(pipeline)
            predicted_frame = detect_and_plot(frame, model)
            if predicted_frame is not None:
                # Convert the predicted frame to a ROS image message and publish it
                ros_image = bridge.cv2_to_imgmsg(predicted_frame, "bgr8")
                pub.publish(ros_image)
    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()
