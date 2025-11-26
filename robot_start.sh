#!/bin/bash
source /opt/ros/galactic/setup.bash
source /home/robot/ros2_ws/install/setup.bash

# Optional: preload libgomp to fix PyTorch TLS issue
#export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
# Start Astra Camera
ros2 launch astra_camera astra.launch.xml &
astra_pid=$!

# Wait a few seconds for the camera to initialize
sleep 5

# Start image_analyze pub
ros2 run image_analyze pub &
pub_pid=$!

sleep 5

ros2 launch rplidar_ros rplidar_a1_launch.py &
rplidar_pid=$!

# Start image_analyze yolo
#ros2 run image_analyze yolo &
#yolo_pid=$!

sleep 5

wait $astra_pid $pub_pid $rplidar_pid 
# Start image_analyze pc (uncomment if needed)
# ros2 run image_analyze pc &
# pc_pid=$!
 
