#!/usr/bin/env bash

source ~/JayBotsDT/jaybot_ws/devel/setup.bash

export ROS_IP=192.168.0.5
export ROS_MASTER_URI=http://192.168.0.3:11311

exec "$@"
