#!/usr/bin/env bash
set -e
apt-get update
apt-get install -y \
  ripgrep \
  python3-rosdep \
  python3-colcon-common-extensions \
  libeigen3-dev \
  libboost-all-dev \
  libceres-dev \
  libopencv-dev \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-proc \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update
