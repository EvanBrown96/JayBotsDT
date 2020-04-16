FROM ros:latest
RUN apt-get update && apt-get install -y ros-melodic-rosbridge-suite
WORKDIR ~/JayBotsDT/jaybot_ws
RUN catkin_make
