from dronekit import connect, VehicleMode, LocationGlobalRelative
import logging
import time
import cv2
import cv2.aruco as aruco
import atexit
import subprocess 

# Define the function to run the YOLO script
def run_yolo_script():
    command = ['python', '/home/avichal/final_yolo/yolov5-master/detect.py']
    subprocess.call(command)

# Register the function to be called at exit
atexit.register(run_yolo_script)

# Connect to the drone (in this case sitl)
vehicle = connect('udp::14550', wait_ready=True)

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

# Fly the drone to 20 meters height
target_altitude = 20  # meters
arm_and_takeoff(target_altitude)

# Move the drone to a specific location (for example)
target_location = LocationGlobalRelative(-35.3632689, 149.1652301, target_altitude)
vehicle.simple_goto(target_location)

# Wait for some time for the drone to reach the destination
time.sleep(30)  # Adjust time as needed

# Run the YOLO script for object detection
run_yolo_script()

# Open a frame to display the detected objects (if required)
# Insert code here to open a frame and display the detected objects using OpenCV

