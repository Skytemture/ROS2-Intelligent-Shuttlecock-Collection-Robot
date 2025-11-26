 LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 ros2 run image_analyze yolo

 ros2 launch rplidar_ros rplidar_a1_launch.py
 
 ros2 launch slam slam.launch.py


sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable robot_start.service


sudo systemctl start robot_start.service

sudo systemctl restart robot_start.service

sudo systemctl status robot_start.service


find ~/ -name "robot_map.*"

source install/setup.bash

localization mode need init_pose
