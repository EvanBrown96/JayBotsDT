export ROS_IP=$(ip addr show wifi0 | grep -Po 'inet \K[\d.]+')
