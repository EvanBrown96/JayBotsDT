sudo apt update
sudo apt full-upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt install ros-melodic-desktop-full
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /home/pmr/JayBotsDT/jaybot_ws/devel/setup.bash" >> /.bashrc
sudo rosdep init
rosdep update
sudo rosdep init
rosdep update