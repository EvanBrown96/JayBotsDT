source setup.bash
# sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'
# sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
sudo dpkg -i --force-all /var/cache/apt/archives/linux-firmware-raspi2_1.20190819-0ubuntu0.18.04.1_armhf.deb
sudo apt -y full-upgrade
sudo apt -y install xrdp openssh-server ros-melodic-rplidar-ros
pip install gpiozero pigpio
sudo systemctl enable ssh.service
sudo systemctl start ssh.service
sudo dpkg-reconfigure openssh-server
echo "source ~/JayBotsDT/env_files/set_env_rpi.bash" >> ~/.bashrc
echo "sudo chmod 666 /dev/ttyUSB0" >> ~/.bashrc
