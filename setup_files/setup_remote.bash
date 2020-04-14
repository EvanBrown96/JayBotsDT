source setup.bash
#echo "source $HOME/JayBotsDT/remote_ws/devel/setup.bash" >> ~/.bashrc
sudo apt -y install ros-melodic-rosbridge-suite ros-melodic-hector-slam
echo "source ~/env_files/set_ros_ip.bash" >> ~/.bashrc
echo "source ~/env_files/set_ros_master_uri.bash" >> ~/.bashrc
