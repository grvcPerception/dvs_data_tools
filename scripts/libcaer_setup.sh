#!/usr/bin/env bash

# @file:   libcaer_setup.sh
# @author: rautaplop
# @brief:  Script to install libcaer.

# libcaer is a C library that allows to use the DVS and the DAVIS event camera.
# Original project is stored at https://gitlab.com/inivation/libcaer

###########################################
# UPDATE AND UPGRADE                      #
###########################################
sudo apt-get -y update
sudo apt-get -y upgrade


###########################################
# REQUIREMENTS                            #
###########################################
sudo apt-get -y install build-essential cmake pkg-config libusb-1.0-0-dev git


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
# INSTALL LIBSERIALPORT                   #
###########################################
git clone git://sigrok.org/libserialport
cd libserialport
./autogen.sh
./configure
make
sudo make install
cd ..


###########################################
# INSTALL LIBCAER                         #
###########################################
git clone https://github.com/inivation/libcaer
cd libcaer
cmake -DCMAKE_INSTALL_PREFIX=/usr . -DENABLE_SERIALDEV=1 -DENABLE_OPENCV=1
make
sudo make install
cd ..


###########################################
# REMOVE TEMPORAL FOLDERS                 #
###########################################
sudo rm -r libserialport
sudo rm -r libcaer


