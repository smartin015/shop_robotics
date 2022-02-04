# syntax=docker/dockerfile:1.3-labs
FROM cyberbotics/webots:R2022a-ubuntu20.04

ENV ROS_DISTRO=galactic
RUN <<EOM
  apt-get update && apt-get -y install --no-install-recommends git curl gnupg2 lsb-release python3-pip &&

  echo "Installing ROS2"
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
  sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

  echo "Installing ROS2, colcon, other deps"
  apt-get update && apt-get install -y --no-install-recommends \
      ros-$ROS_DISTRO-ros-base \
      python3-colcon-common-extensions \
      libzmq3-dev 
    # ros-galactic-webots-ros2 \
 
  pip3 install pyzmq

  echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /root/.bashrc

  # echo "Installing MoveIt!"
  #RUN apt-get update && apt-get install -y --no-install-recommends curl build-essential cmake python3-vcstool python3-colcon-common-extensions && \
  #  mkdir -p /moveit/src && cd /moveit/src && \
  #  curl https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos -o moveit2.repos && \
  #  vcs import < moveit2.repos && \
  #  export COLCON_WS=/moveit/ && \
  #  rosdep update && \
  #  rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y && \

  echo "Installing GPU acceleration libs"
  # http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
  # TODO accept ARGs to select particular GPU type
   DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends libgl1-mesa-glx libgl1-mesa-dri libnss3 libasound2 wget software-properties-common &&
  rm -rf /var/lib/apt/lists/*
EOM

# Copy in our webots config
COPY config/Webots.conf /root/.config/Cyberbotics/

ENV ROS2PKG=shop_robotics_sim
COPY . /$ROS2PKG
WORKDIR /$ROS2PKG
# Switch to using bash as shell so ROS binaries are in $PATH
SHELL ["bash", "-c"] 

ENV ROS2NODES="sim moveit servo"
RUN <<EOM
  echo "== Building the ROS2 package $ROS2PKG =="
  colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

  echo "Linking package into .bashrc"
  echo "source \"/$ROS2PKG/install/setup.bash\"" >> /root/.bashrc
EOM

# Required for ros2 environment to also find the webots packages
# https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux&tab-language=python#environment-variables
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller
ENV PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python38
