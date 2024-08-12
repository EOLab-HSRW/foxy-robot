# use the official ubuntu 22.04 image
FROM ubuntu:22.04

# setting an enviromental viriable for noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# update package index and other installation dependecies

RUN apt update && \
    apt install -y locales curl gnupg2 lsb-release build-essential && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    # Ensure that the ubuntu Universal repository is enabled
    apt install software-properties-common && \
    add-apt-repository universal

# Add the ROS 2 GPG key with apt
# Add the repository to source list

RUN apt update && \
    curl curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update & upgrade the apt repository caches 
RUN apt update && apt upgrade && \
    apt install -y ros-humble-desktop


# Source the ros 2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

CMD [ "/bin/bash" ]
