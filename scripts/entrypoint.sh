#!/bin/bash

# Source the workspace
. /root/camco_ws/install/setup.bash

#If the container is running on a server handling the navigation and the web gui
if [ "$1" = "nav-server" ]; then
    ros2 launch camco_mission camco_mission.launch.py map:="$2" gui:=true&
    python3 -m http.server -d /root/camco_ws/src/Campus-Courier/camco_frontend -b 0.0.0.0 8080

#If the container is running on the robot's companion computer that is handling the drivers only
elif [ "$1" = "nav-robot" ]; then
    echo "Make sure to add the Udev rules for the RPLIDAR and the Kobuki as specified in README.md"
    ros2 launch camco_launch camco.launch.py use_rviz:=false

#If the container is running on the robot's companion computer that is handling the drivers, navigation, and web gui
elif [ "$1" = "nav-server-robot" ]; then
    echo "Make sure to add the Udev rules for the RPLIDAR and the Kobuki as specified in README.md"
    ros2 launch camco_launch camco.launch.py use_rviz:=false
    ros2 launch camco_mission camco_mission.launch.py map:="$2" gui:=true &
    python3 -m http.server -d /root/camco_ws/src/Campus-Courier/camco_frontend -b 0.0.0.0 8080

#If the container is running on the robot's companion computer that is handling the drivers, navigation, and web gui
elif [ "$1" = "slam" ]; then
    ros2 launch camco_navigation slam.launch.py

else
    echo "Invalid option: $1 options are: nav-server, nav-robot, nav-server-robot, or slam"
    exit 1
fi