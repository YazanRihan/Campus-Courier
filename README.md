# Campus-Courier

To build Campus Courier on Foxy

```
cd ~/camco_ws
vcs import src < src/Campus-Courier/dependencies.repos
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