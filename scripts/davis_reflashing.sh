#!/usr/bin/env bash

# @file   davis_reflashing.sh
# @author rautaplop
# @brief  Script to upgrade DAVIS event camera firmware using flashy.

###########################################
# UPDATE AND UPGRADE                      #
###########################################
sudo apt update
sudo apt upgrade


###########################################
# GET FLASHY                              #
###########################################
wget http://release.inivation.com/flashy/flashy-linux-1.4.2.tar.gz #TODO: Get last release???
tar -xzf flashy-linux-1.4.2.tar.gz


###########################################
# EXECUTE FLASHY                          #
###########################################
cd flashy-1.4.2/bin
chmod +x flashy
./flashy


###########################################
# REMOVE TEMPORAL FILES                   #
###########################################
cd ../..
sudo rm flashy-linux-1.4.2.tar.gz
sudo rm -r flashy-1.4.2
