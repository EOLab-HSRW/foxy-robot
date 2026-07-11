# Installation

If you've never done it before, setting up a working environment for a robot can be a major headache, so we provide an easy way to set up the working environment with our `a2s` tool.

## 📋 Requirements

- **Operative system**: [Ubuntu 22.04 Desktop](https://releases.ubuntu.com/jammy/)
- **Architecture**: amd64 or arm64

The rationale behind these requirements:
1. The world of robotics runs essentially on GNU/Linux based operating systems, so if you want to develop in the field what better than start using GNU/Linux?
2. We want to enforce Tier 1 support. To minimize dependency problems with packages distributed as pre-compiled binaries we are going to stick to [REP-2000](https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027) and only support the above mentioned operating system and architectures.

Comments in possible scenarios:
- Can I use dual-boot? yes.
- Can I use Windows? yes, as long as you use Ubuntu 22.04 with [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) on Windows 11 with a version equal or higher than 22H2. This is important for rendering Ubuntu graphics applications (like the simulator) on your Windows desktop.
- Can I use a virtual machine? Technically yes but it is very problematic with the graphics drivers and you will experience problems with the simulations so it is not recommended to use virtual machine. But if you still want to try, go ahead and good luck.
- Can I use a Mac: No! 😅.

## Manual Install

First let remove any Gazebo Fortress and install Gazebo Harmonic:

```sh
sudo apt remove ignition* ros-humble-ros-ign* && sudo apt autoremove

sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

sudo apt-get install ros-humble-ros-gzharmonic
```

install our drivers:

```
sudo apt install foxy-drivers

```

Setup workspace:

```sh
mkdir -p ~/foxy_ws/src
cd ~/foxy_ws/src
git clone https://github.com/EOLab-HSRW/foxy-robot/
cd ~/foxy_ws
source /opt/ros/humble/setup.bash

# in some cases you need to init the rosdep
sudo rosdep init
rosdep update
```

Setup ROS 2 Humble + ROS 2 Control + Gazebo Harmonic:
```sh
cd ~/foxy_ws/src
git clone https://github.com/ros-controls/gz_ros2_control -b humble
cd ~/foxy_ws/
export GZ_VERSION=harmonic
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys="ros_gz_bridge ros_gz_sim"
colcon build --packages-up-to gz_ros2_control --symlink-install
colcon build --symlink-install
source install/setup.bash
```

```
ros2 launch foxy_bringup start.launch.py
```

## Optimal

```
wget -O /tmp/eolab-drones-sources.deb \
  https://github.com/EOLab-HSRW/drones-sources/releases/latest/download/drones-sources_latest.deb
sudo apt-get install /tmp/eolab-drones-sources.deb
sudo apt install ros-humble-foxy-bringup-sim
```


## Recommended

```
vcs import < .repos
```

1. Install our container management system:

> [!CAUTION]
> Be aware: Running commands like the following **is extremely dangerous**, it is a gateway to running malicious code on your computer. You should never copy and paste commands you see on the internet without first inspecting them carefully and making sure you know what they do.

For transparency: to make things more convenient we provide a script called [install.sh](https://raw.githubusercontent.com/harleylara/a2s-cli/refs/heads/main/install.sh) (as you can see in the url) which takes care of installing [apptainer](https://apptainer.org/), nvidia-container-toolkit and the necessary dependencies to run our container management tool (`a2s`). You are invited to read our script to verify that there is no malicious code getting into your computer.

```sh
wget --show-progress --tries=1 -qO- https://raw.githubusercontent.com/harleylara/a2s-cli/refs/heads/main/install.sh | bash
```

2. Clone robot repo:

```sh
mkdir -p ~/foxy_robot_ws/src && cd ~/foxy_robot_ws/src
git clone https://github.com/EOLab-HSRW/foxy-robot/ && cd foxy-robot
```

3. Initialize the container.

This may take a few minutes, but it is a one-time operation. It will create a container with ROS already installed and includes all the dependencies necessary to operate the foxy robot.

```sh
a2s init foxy
```

Done.

## Host System

In case you want a more manual installation, first make sure you have installed [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) correctly and proceed with the following steps:

> [!NOTE]
> Note that installing ROS directly on your computer can cause conflicts with dependencies, a good example of this is anaconda/rize/some-others when they take control of the python interpreter this generates problems with ROS. Therefore if you opt for this means of installation you will have to handle these situations on your own as they are specific to your system and difficult to replicate to provide a possible solution.

The next set of commands will clone our repo and install the dependencies for you:

```sh
mkdir -p ~/foxy_robot_ws/src && cd ~/foxy_robot_ws/src
git clone https://github.com/EOLab-HSRW/foxy-robot/
cd ~/foxy_dev_ws
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
