mkdir -p ~/foxy_ws/src
cd ~/foxy_ws/src

git clone https://github.com/EOLab-HSRW/foxy-robot.git --depth 1

cd ~/foxy_ws
vcs import src --shallow < ~/foxy_ws/src/foxy-robot/hw.repos

# required for gscam
sudo apt install --no-install-recommends -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools

source /opt/ros/humble/setup.bash
sudo apt update
rosdep update

rosdep install \
  --from-paths src/gscam \
  --ignore-src

colcon build \
  --packages-select gscam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

rosdep install -r -y --ignore-src \
  --from-paths $(colcon list --paths-only --packages-up-to foxy_bringup_hw)

colcon build --packages-up-to foxy_bringup_hw
