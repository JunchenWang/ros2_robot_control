#!/bin/bash

# 检查脚本是否以 root 用户权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 用户权限运行此脚本: sudo $0"
    exit 1
fi

# 安装ROS2
wget http://fishros.com/install -O fishros && . fishros

# 安装ROS2控制
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install qt6-base-dev qt6-charts-dev libjsoncpp-dev

# 安装rqt中的tf tree插件
sudo apt install -y ros-jazzy-rqt-tf-tree
rm -rf ~/.config/ros.org/rqt_gui.ini