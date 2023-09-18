#!/bin/bash

sudo apt-get update
# CUDA download for debian
#wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-debian11-11-7-local_11.7.0-515.43.04-1_amd64.deb
#sudo dpkg -i cuda-repo-debian11-11-7-local_11.7.0-515.43.04-1_amd64.deb
#sudo cp /var/cuda-repo-debian11-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/
#sudo add-apt-repository contrib
#sudo apt-get -y install cuda
sudo apt-get install -y libcgal-dev
sudo apt-get install -y libx11-dev
sudo apt-get install -y libxrandr-dev
sudo apt-get install -y libxinerama-dev
sudo apt-get install -y libxcursor-dev
sudo apt-get install -y libxi-dev
sudo apt-get install -y libgl1-mesa-dev
