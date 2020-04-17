source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_IP=$(ip addr show eth1 | grep -Po 'inet \K[\d.]+')
export ROS_MASTER_URI=http://$ROS_IP:11311
