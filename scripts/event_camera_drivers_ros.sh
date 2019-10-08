#!/usr/bin/env bash

# @file:   event_camera_driver_ros.sh
# @author: rautaplop
# @brief:  Script to install the uzh-rpg ROS event camera driver.

# This script uses the installation steps published in https://github.com/uzh-rpg/rpg_dvs_ros
# Select device: davis, dvs
CAMERA=davis

###########################################
# UPDATE AND UPGRADE                      #
###########################################
sudo apt update
sudo apt upgrade


###########################################
# INSTALL DEPENDENCIES                    #
###########################################
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install ros-kinetic-camera-info-manager
sudo apt-get install ros-kinetic-image-view

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools


###########################################
# UPGRADE CMAKE                           #
###########################################
#TODO: Not install if already have an appropriate version.
wget https://github.com/Kitware/CMake/releases/download/v3.14.4/cmake-3.14.4.tar.gz
tar -xzf cmake-3.14.4.tar.gz
cd cmake-3.14.4
./bootstrap --prefix=$HOME/cmake-install
make -j$(nproc)
make install
export PATH=$HOME/cmake-install/bin:$PATH
export CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH
echo "" >> ~/.bashrc
echo "#CMAKE" >> ~/.bashrc
echo "export PATH=$HOME/cmake-install/bin:$PATH" >> ~/.bashrc
echo "CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH" >> ~/.bashrc
cd ..
sudo rm cmake-3.14.4.tar.gz
sudo rm -r cmake-3.14.4


###########################################
# CREATE WORKSPACE                        #
###########################################
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin config --init --mkdirs --extend /opt/ros/kinetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release


###########################################
# CLONE REPOSITORIES                      #
###########################################
cd ~/catkin_ws/src
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/uzh-rpg/rpg_dvs_ros.git


###########################################
# BUILD PACKAGES                          #
###########################################
catkin build ${CAMERA}_ros_driver


###########################################
# INSTALL SCRIPT IN LIBCAER_CATKIN        #
###########################################
source ~/catkin_ws/devel/setup.bash
roscd libcaer_catkin
sudo ./install.sh


###########################################
# TEST INSTALLATION                       #
###########################################
catkin build dvs_renderer
source ~/catkin_ws/devel/setup.bash
roslaunch dvs_renderer davis_mono.launch


