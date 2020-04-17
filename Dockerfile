FROM ros:latest
RUN apt-get update && apt-get install -y ros-melodic-rosbridge-suite iproute2 openssh-server
WORKDIR ~
COPY remote_ws remote_ws
COPY env_files env_files
WORKDIR remote_ws
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash ; catkin_make'
ENTRYPOINT ["/bin/bash", "-c", "source ../env_files/remote_startup.bash ; echo $ROS_IP ; roslaunch remote_app standard.launch"]
