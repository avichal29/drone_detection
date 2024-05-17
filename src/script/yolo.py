import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
from threading import Thread

# Initialize ROS node
rospy.init_node('object_detection')

# Initialize CvBridge
bridge = CvBridge()

# Define the function to run the YOLO script
def run_yolo_script():
    command = ['python', '/home/avichal/final_yolo/yolov5-master/detect.py', '--source', '/camera_head/image_raw']
    subprocess.call(command)

# Function to process detected objects
def process_detected_objects(frame):
    # Process the detected objects as needed
    # For example, draw bounding boxes, display information, etc.
    pass

# Callback function for processing camera image
def image_callback(img_msg):
    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    
    # Run YOLO script in a separate thread
    yolo_thread = Thread(target=run_yolo_script)
    yolo_thread.start()

    # Process the detected objects
    process_detected_objects(cv_image)

    # Display the camera feed
    cv2.imshow('Camera Feed', cv_image)
    cv2.waitKey(1)

# Subscribe to the camera image topic
sub_image = rospy.Subscriber("/camera_head/image_raw", Image, image_callback)

# Spin ROS node
rospy.spin()

