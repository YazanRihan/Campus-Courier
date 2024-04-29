# Campus-Courier

To build Campus Courier on Foxy

```
cd
mkdir -p camco_ws/src
cd ~/camco_ws/src
git clone https://github.com/YazanRihan/Campus-Courier.git
cd ~/camco_ws
vcs import src < src/Campus-Courier/dependencies.repos
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

To build Campus Courier on humble

```
cd
mkdir -p camco_ws/src
cd ~/camco_ws/src
git clone https://github.com/YazanRihan/Campus-Courier.git
cd ~/camco_ws
vcs import src < src/Campus-Courier/dependencies_humble.repos
rosdep update
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

To connect to RPLIDAR
```
cd ~/camco_ws/src/Campus-Courier/scripts/
sudo cp rplidar.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
```

To run the Kobuki and RPLIDAR on boot
```
cd ~/camco_ws/src/Campus-Courier/scripts/
sudo cp camco_launch.service /etc/systemd/system
sudo chmod 664 /etc/systemd/system/camco_launch.service
sudo systemctl daemon-reload
sudo systemctl start camco_launch.service
sudo systemctl enable camco_launch.service

#Check status by
sudo systemctl status camco_launch.service

#To restart use
sudo systemctl restart camco_launch.service
```