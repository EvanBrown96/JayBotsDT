source setup.bash
#echo "source $HOME/JayBotsDT/remote_ws/devel/setup.bash" >> ~/.bashrc
sudo apt -y install ros-melodic-rosbridge-suite ros-melodic-hector-slam
echo "export ROS_IP=\$(ip addr show wifi0 | grep -Po 'inet \K[\d.]+')" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://\$ROS_IP:11311" >> ~/.bashrc
