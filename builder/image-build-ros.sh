#! /usr/bin/env bash

#
# Script for building ROS packages from scratch
#
# Copyright (C) 2022 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -ex # exit on error, echo commands

# http://wiki.ros.org/Installation/Source

ROS_DISTRO=noetic
. /etc/os-release # set $VERSION_CODENAME to Debian release code name
export ROS_OS_OVERRIDE=debian:$VERSION_CODENAME

echo "=== Building ROS from scratch"

#echo "--- Adding sources"
#echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
#curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

apt-get update
apt-get install -y python3-distutils python3-rosdep python3-rosinstall-generator build-essential # python3-vcstool

# install vcstool using pip
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && rm get-pip.py
pip3 install -U vcstool

# sudo rosdep init
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update

# rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init
# rosdep --os=debian:$VERSION_CODENAME update

echo "--- Create Catkin workspace to build ROS package"
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "--- Download ROS sources"
rosinstall_generator ros_base --rosdistro $ROS_DISTRO --deps --tar > noetic.rosinstall
mkdir ./src
vcs import --input noetic.rosinstall ./src

# https://answers.ros.org/question/343367/catkin-package-dependencies-issue-when-installing-ros-melodic-on-raspberry-pi-4/
#sudo apt remove python-rospkg
#sudo apt remove python-catkin-pkg
##sudo apt --fix-broken install
#sudo apt-get autoremove

echo "--- Install catkin_pkg"
cd
https://github.com/ros-infrastructure/catkin_pkg.git
cd catkin_pkg
python3 setup.py install
cd ~/ros_catkin_ws

echo "--- Resolve dependencies"
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO -y --os=debian:$VERSION_CODENAME

echo "--- Build ROS"
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

source ~/ros_catkin_ws/install_isolated/setup.bash
