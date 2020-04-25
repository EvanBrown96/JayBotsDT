source /opt/ros/melodic/setup.bash
source ~/JayBotsDT/jaybot_ws/devel/setup.bash
export ROS_IP=$(ip addr show wlan0 | grep -Po 'inet \K[\d.]+')
