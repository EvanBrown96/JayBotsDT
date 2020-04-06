sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt -y update
sudo apt -y full-upgrade
sudo apt -y install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep python-pip xrdp
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /home/pmr/JayBotsDT/jaybot_ws/devel/setup.bash" >> ~/.bashrc
sudo rosdep init
rosdep update
pip install gpiozero pigpio
