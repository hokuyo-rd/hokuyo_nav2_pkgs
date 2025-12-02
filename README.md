# hokuyo_nav2_pkgs

ROS2 packages of Navigation system for outdoor mobile robot,
by using navigation2 for hokuyo RSF (fusion sensor) consisted by hokuyo YVT-35LX and RTK-GNSS.

# Setup

```bash
sudo apt-get update 

cd YOUR_ROS2_WS
git clone --recursive -b https://github.com/hokuyo-rd/hokuyo_nav2_pkgs.git hokuyo_navigation2
rosdep update
rosdep install --from-paths src/hokuyo_navigation2 --ignore-src -r -y
colcon build
colcon build --symlink-install --packages-select hokuyo_navigation2 lio_nav2_bringup simple_fastlio_localization icart_mini_driver
```