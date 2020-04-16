export ROS_IP=$(ip addr show wlan0 | grep -Po 'inet \K[\d.]+') || export ROS_IP=$(ip addr show eth0 | grep -Po 'inet \K[\d.]+')
