#!/bin/bash

# 检查是否使用root权限运行
if [ "$EUID" -ne 0 ]; then
  echo "请以root权限运行此脚本！"
  exit 1
fi

# 添加'realtime'用户组
groupadd realtime || echo "用户组 'realtime' 已存在，跳过创建。"

# 将当前用户添加到'realtime'组
usermod -aG realtime $(whoami)
echo "当前用户 $(whoami) 已被添加到'realtime'组。"

# 配置/etc/security/limits.conf
LIMITS_CONF="/etc/security/limits.conf"
if ! grep -q "@realtime" "$LIMITS_CONF"; then
  echo "添加实时调度限制到 $LIMITS_CONF ..."
  cat >> "$LIMITS_CONF" <<EOL

# Realtime group settings
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOL
  echo "实时调度限制已成功添加。"
else
  echo "$LIMITS_CONF 已包含实时调度限制，跳过此步骤。"
fi

echo "注意：您需要注销并重新登录，或者重启系统，以使新配置生效。"
