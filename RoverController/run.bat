wsl source /opt/ros/melodic/setup.bash ; export ROS_HOSTNAME=$(hostname) ; export ROS_MASTER_URI=http://$(hostname):11311 ; echo $ROS_MASTER_URI ; roslaunch rosbridge_server rosbridge_websocket.launch
RoverController/bin/Debug/RoverController.exe
