#!/bin/bash
# chmod +x Test_pwm.sh
cd ../..

sudo -i
source /opt/ros/jazzy/setup.bash
cd /home/rasp/WS_AWAKE

#colcon build
source install/setup.bash

ros2 launch ./src/AWAKE/launch.launch.py