clone the repository
then go to src folder
run the scripts for the movement in the script folder 
open the world using the present inside the yolo_detection and then world

run the sitl 
~/ardupilot/ArduCopter$ cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console

launch the gazebo world
roslaunch gazebo_ros empty_world.launch world_name:=/home/avichal/ros_workspace/src/yolo_detection/world/custom.world

then run the script from the script folder
