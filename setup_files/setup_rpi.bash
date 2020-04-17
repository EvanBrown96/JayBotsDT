sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt -y update
sudo apt -y full-upgrade
sudo apt -y install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip xrdp openssh-server ros-melodic-rplidar-ros

sudo dpkg -i --force-all /var/cache/apt/archives/linux-firmware-raspi2_1.20190819-0ubuntu0.18.04.1_armhf.deb
sudo apt -y full-upgrade

sudo rosdep init
rosdep update

pip install gpiozero pigpio

sudo systemctl enable ssh.service
sudo systemctl start ssh.service
sudo dpkg-reconfigure openssh-server

cd ~/JayBotsDT/jaybot_ws
catkin_make

sudo echo "#!/bin/bash" > /etc/rc.local
sudo echo 'chmod 666 /dev/ttyUSB0' >> /etc/rc.local
sudo chmod +x /etc/rc.local

echo "source ~/JayBotsDT/env_files/rpi_startup.bash" >> ~/.bashrc
echo "echo Don't forget to set ROS_MASTER_URI !!!" >> ~/.bashrc
