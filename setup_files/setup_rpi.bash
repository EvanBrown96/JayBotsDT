sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt -y update
sudo apt -y full-upgrade
sudo apt -y install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip xrdp openssh-server ros-melodic-rplidar-ros

sudo dpkg -i --force-all /var/cache/apt/archives/linux-firmware-raspi2_1.20190819-0ubuntu0.18.04.1_armhf.deb
sudo apt -y full-upgrade

echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/30-ubiquity.list

sudo rosdep init
rosdep update

pip install gpiozero pigpio

sudo systemctl enable ssh.service
sudo systemctl start ssh.service
sudo dpkg-reconfigure openssh-server

cd ~/JayBotsDT/jaybot_ws
rosdep install --from-paths src --ignore-src -y
catkin_make

echo "#!/bin/bash" | sudo tee /etc/rc.local
echo 'chmod 666 /dev/ttyUSB0' | sudo tee -a /etc/rc.local
sudo chmod +x /etc/rc.local

echo "source ~/JayBotsDT/env_files/rpi_startup.bash" >> ~/.bashrc
echo "echo Don't forget to set ROS_MASTER_URI !!!" >> ~/.bashrc
