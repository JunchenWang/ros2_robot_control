#!/bin/bash
# 请用 sudo -E 运行此脚本以保持环境变量

# 检查脚本是否以 root 用户权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 用户权限运行此脚本: sudo $0"
    exit 1
fi

cd ~/Desktop
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
sudo make install
rm -rf ~/Desktop/ur_rtde

sudo apt-get update
sudo apt-get install ros-jazzy-ur

echo "UR RTDE 安装完成"