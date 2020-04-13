start %~dp0/RoverController/bin/Debug/RoverController.exe
wsl source /opt/ros/melodic/setup.bash ; export ROS_HOSTNAME=$(ip addr show wifi0 ^| grep -Po 'inet \K[\d.]+') ; export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311 ; echo $ROS_MASTER_URI ; roslaunch rosbridge_server rosbridge_websocket.launch
