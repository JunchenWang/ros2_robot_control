#!/bin/bash
# 请用 sudo -E 运行此脚本以保持环境变量

# 检查脚本是否以 root 用户权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 用户权限运行此脚本: sudo $0"
    exit 1
fi

# 下载并解压 Force Dimension SDK
cd ~/Desktop
wget https://www.forcedimension.com/downloads/sdk/sdk-3.17.6-linux-x86_64-gcc.tar.gz
tar -xvf sdk-3.17.6-linux-x86_64-gcc.tar.gz
# 如果没有 ~/libs 目录，创建一个
mkdir -p ~/libs
mv sdk-3.17.6 ~/libs
mv ~/libs/sdk-3.17.6 ~/libs/forcedimension
rm -rf sdk-3.17.6-linux-x86_64-gcc.tar.gz
rm -rf sdk-3.17.6-linux-x86_64-gcc

# 安装 Force Dimension SDK 依赖
sudo apt install -y libusb-1.0-0-dev

# 建立软链接
ln -s ~/libs/forcedimension/lib/release/lin-x86_64-gcc/libdrd.so.3.17.6 ~/libs/forcedimension/lib/release/lin-x86_64-gcc/libdrd.so.3
ln -s ~/libs/forcedimension/lib/release/lin-x86_64-gcc/libdhd.so.3.17.6 ~/libs/forcedimension/lib/release/lin-x86_64-gcc/libdhd.so.3

# 定义要添加的行
EXPORT_LINE="export LD_LIBRARY_PATH=~/libs/forcedimension/lib/release/lin-x86_64-gcc:\$LD_LIBRARY_PATH"

# 检查是否已经存在
if ! grep -Fxq "$EXPORT_LINE" ~/.bashrc; then
    echo "添加 LD_LIBRARY_PATH 到 ~/.bashrc..."
    echo "$EXPORT_LINE" >> ~/.bashrc
else
    echo "LD_LIBRARY_PATH 已经存在于 ~/.bashrc 中，无需重复添加。"
fi

# 定义规则文件路径和内容
RULES_FILE="/etc/udev/rules.d/99-usb.rules"
RULES_CONTENT='SUBSYSTEM=="usb", ATTRS{idVendor}=="1451", ATTRS{idProduct}=="0402", MODE="0666"'

# 创建或更新规则文件
if [[ -f $RULES_FILE ]]; then
    if grep -Fxq "$RULES_CONTENT" "$RULES_FILE"; then
        echo "规则已存在，无需重复添加。"
    else
        echo "添加规则到 $RULES_FILE..."
        echo "$RULES_CONTENT" >> "$RULES_FILE"
    fi
else
    echo "创建规则文件 $RULES_FILE 并添加规则..."
    echo "$RULES_CONTENT" > "$RULES_FILE"
fi

# 重启 udev 服务以应用更改
echo "重新加载 udev 规则..."
udevadm control --reload
udevadm trigger