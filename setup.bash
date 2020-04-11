sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'
# sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
sudo apt -y update
sudo dpkg -i --force-all /var/cache/apt/archives/linux-firmware-raspi2_1.20190215-0ubuntu0.18.04.1_armhf.deb
sudo apt -y full-upgrade
sudo apt -y install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep python-pip xrdp
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /home/pmr/JayBotsDT/jaybot_ws/devel/setup.bash" >> ~/.bashrc
echo "sudo chmod 666 /dev/ttyUSB0" >> ~/.bashrc
sudo rosdep init
rosdep update
pip install gpiozero pigpio
cd JaybotsDT/jaybot_ws
git clone https://github.com/Slamtec/rplidar_ros src/rplidar_ros
catkin_make
sudo apt install openssh-server
sudo systemctl enable ssh.service
sudo systemctl start ssh.service
sudo dpkg-reconfigure openssh-server