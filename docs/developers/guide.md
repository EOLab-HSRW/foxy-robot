# Developer Guide

- [ROS-Robot Package structure](https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html)

## Manual Setup Software in Robot

```bash
mkdir -p ~/foxy_ws/src/ && cd ~/foxy_ws/src
git clone https://github.com/EOLab-HSRW/foxy-robot -b working-in-hardware
cd ~/foxy_ws
rosdep install -r -y --ignore-src \
  --from-paths $(colcon list --paths-only --packages-up-to foxy_bringup_hw)
colcon build --packages-up-to foxy_bringup_hw
```

for dev machine:
```
rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --packages-up-to foxy_bringup_sim
```

for hardware only:
```
rosdep install -r -y --ignore-src \
  --from-paths $(colcon list --paths-only --packages-up-to foxy_bringup_hw)
colcon build --packages-up-to foxy_bringup_hw
  --from-paths $(colcon list --paths-only --packages-up-to foxy_bringup_hw)
```

## Start

```
ros2 launch foxy_bringup start.launch.py mode:=hw
```
