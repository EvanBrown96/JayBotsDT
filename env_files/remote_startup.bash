source /opt/ros/melodic/setup.bash
source /root/remote_ws/devel/setup.bash
export ROS_IP=$(ip addr show eth2 | grep -Po 'inet \K[\d.]+')
export ROS_MASTER_URI=http://$ROS_IP:11311
