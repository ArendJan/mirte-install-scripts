#!/bin/bash
set -xe
# Install vcstool
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update || true
sudo apt install -y python3-vcstool

# Download all Mirte repositories
vcs import --workers 1 <repos.yaml #TODO: get yaml file as parameter

# TODO: set remote to gitlab when checkout from local
