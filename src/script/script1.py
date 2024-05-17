from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import subprocess
from threading import Thread

bridge = CvBridge()

# Define the function to run the YOLO script
def run_yolo_script():
    command = ['python3', '/home/avichal/final_yolo/yolov5-master/detect.py', '--source', '/camera_name/image_raw', '--device', 'cpu']
    subprocess.call(command)

# Connect to the drone
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
    while not vehicle.is_armable:
        print("Waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")  # Set guided mode of drone
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting to arm")
        time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= target_altitude * 0.95:  # Check if target altitude reached
            print("Target altitude reached")
            break
        time.sleep(1)

# Function to capture video from the drone camera and display it in a frame
def capture_video_and_display(frame):
    show_video = True
    if show_video:
        # --- Display the frame
        cv2.imshow('frame', frame)

        # --- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            exit()

def image_callback(img_msg):
    # Log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Show the converted image
    capture_video_and_display(cv_image)

# Main function
def main():
    # Fly the drone to 10 meters height
    target_altitude = 10  # meters
    arm_and_takeoff(target_altitude)

    # Initialize the ROS node
    rospy.init_node('mavlink_lander', anonymous=True)

    # Subscribe to the camera feed
    sub_image = rospy.Subscriber("/camera_head/image_raw", Image, image_callback)

    # Start YOLO script in a separate thread
    yolo_thread = Thread(target=run_yolo_script)
    yolo_thread.start()

    # Spin to keep the script running
    rospy.spin()

if __name__ == "__main__":
    main()

