#!/usr/bin/env zsh
# 不要开启 -u（nounset）
set -Ee -o pipefail
# 显式关闭 nounset，防外部环境影响
set +u 2>/dev/null || true

echo "启动机器人系统..."
# 可配：包和 launch 文件
LAUNCH_PKG="${LAUNCH_PKG:-node_manager}"
LAUNCH_FILE="${LAUNCH_FILE:-bringup_all_dm_m6.launch.py}"

# ROS 环境
source /opt/ros/humble/setup.zsh


# 工作区
WS="/opt/kernelmind/apex"
cd "$WS"
[ -f install/setup.zsh ] && source install/setup.zsh

echo "启动 Launch..."
exec ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" use_rviz:=true "$@"
