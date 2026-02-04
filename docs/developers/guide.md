# Developer Guide

- [ROS-Robot Package structure](https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html)

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
  --from-paths $(colcon list --paths-only --packages-up-to foxy_bringup_sim)
```
