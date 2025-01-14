#!/bin/bash

# 检查脚本是否以 root 用户权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 用户权限运行此脚本: sudo $0"
    exit 1
fi

# 更新系统软件包列表
echo "更新系统软件包列表..."
sudo apt update -y && sudo apt upgrade -y

# 安装通用依赖
echo "安装通用依赖..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    libusb-1.0-0-dev \
    qt6-base-dev \
    qt6-charts-dev \
    libjsoncpp-dev \
    gparted \
    vim \
    tree \
    dpkg \
    plocate \
    simplescreenrecorder \
    vlc \
    libfftw3-dev \
    gnuplot \