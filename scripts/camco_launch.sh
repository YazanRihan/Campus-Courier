#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/$USER/camco_ws/install/setup.bash
ros2 launch camco_launch camco.launch.py use_rviz:=false