# 进入workspace
cd rush_mog_ws
colcon build --packages-select custom_msgs center_server
source install/setup.bash
ros2 launch center_server center_server.launch.py
