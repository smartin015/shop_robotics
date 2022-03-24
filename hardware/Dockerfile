# syntax=docker/dockerfile:1.3-labs
FROM ros:galactic-ros-base

RUN apt-get update \
  && apt-get -y install --no-install-recommends python3-pip \
  && pip3 install pyzmq pyserial \
  && rm -rf /var/lib/apt/lists*

ENV ROS2PKG=shop_robotics_interface
ENV ROS2NODES="interface"
COPY . /$ROS2PKG
WORKDIR /$ROS2PKG
# Switch to using bash as shell so ROS binaries are in $PATH
SHELL ["bash", "-c"] 

RUN <<EOM
  echo "== Building the ROS2 package $ROS2PKG =="
  colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

  echo "Linking package into .bashrc"
  echo "source \"/$ROS2PKG/install/setup.bash\"" >> /root/.bashrc
EOM
