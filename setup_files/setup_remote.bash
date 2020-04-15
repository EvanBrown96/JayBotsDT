source setup.bash

sudo apt -y install ros-melodic-rosbridge-suite ros-melodic-hector-slam

rm ~/.ssh/known_hosts
ssh-keygen -t rsa

cd ~/JayBotsDT/remote_ws
catkin_make

echo "source ~/JayBotsDT/env_files/set_env_remote.bash" >> ~/.bashrc
