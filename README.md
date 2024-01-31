# Campus-Courier

To build Campus Courier on Foxy

```
cd
mkdir -p camco_ws/src
cd ~/camco_ws/src
git clone https://github.com/YazanRihan/Campus-Courier.git
cd ~/camco_ws
vcs import src < src/Campus-Courier/dependencies.repos
sudo rm -r src/create3_sim/irobot_create_gazebo
sudo rm -r src/create3_sim/irobot_create_gazebo_plugins
sudo rm -r src/create3_sim/irobot_create_sim
sudo rm -r src/create3_sim/irobot_create_toolbox
sudo rm -r src/turtlebot4/turtlebot4_msgs
sudo rm -r src/turtlebot4/turtlebot4_navigation
sudo rm -r src/turtlebot4/turtlebot4_node
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

To connect Kobuki hardware
```
wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules
sudo cp 60-kobuki.rules /etc/udev/rules.d
#   --> failing all else, a reboot will work
sudo service udev reload
sudo service udev restart
```