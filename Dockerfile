FROM ros:melodic
RUN apt-get update && \
    apt-get install -y iproute2 openssh-server python-pip ros-melodic-rosbridge-suite && \
    apt-get install -y python-twisted python-twisted-bin python-twisted-core python-autobahn python-tornado python-backports.ssl-match-hostname python-pil python-bson python-bson-ext
WORKDIR /root

RUN rosdep update

COPY env_files env_files

COPY hector_ws hector_ws
WORKDIR hector_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make'

WORKDIR /root
COPY mexplore-ws mexplore-ws
WORKDIR mexplore-ws
run rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source ../hector_ws/devel/setup.bash && catkin_make'

RUN pip install pathfinding

WORKDIR /root
COPY jaybot_ws jaybot_ws
WORKDIR jaybot_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source ../mexplore-ws/devel/setup.bash && catkin_make'

WORKDIR /root
COPY remote_ws remote_ws
WORKDIR remote_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source ../jaybot_ws/devel/setup.bash && catkin_make'

ENTRYPOINT ["/bin/bash", "-c", "source ../env_files/remote_startup.bash ; echo $ROS_IP ; roslaunch remote_app standard.launch"]
