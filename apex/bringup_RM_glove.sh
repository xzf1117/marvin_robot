#!/usr/bin/env bash
# 不要开启 -u（nounset）
set -Ee -o pipefail
# 显式关闭 nounset，防外部环境影响
set +u 2>/dev/null || true

echo "启动机器人系统..."
# 可配：包和 launch 文件
LAUNCH_PKG="${LAUNCH_PKG:-node_manager}"
LAUNCH_FILE="${LAUNCH_FILE:-bringup_all_sharpa_m6_glove.launch.py}"
WS="/home/jjj/code/Apex_Deploy"
LAUNCH_PID=""
CLEANING_UP=0

remove_path_entries_matching() {
    local var_name="$1"
    local pattern="$2"
    local old_value="${!var_name:-}"
    local new_value=""
    local entry

    IFS=':' read -ra entries <<< "$old_value"
    for entry in "${entries[@]}"; do
        [ -z "$entry" ] && continue
        case "$entry" in
            *"$pattern"*) continue ;;
        esac
        if [ -z "$new_value" ]; then
            new_value="$entry"
        else
            new_value="$new_value:$entry"
        fi
    done
    export "$var_name=$new_value"
}

kill_bringup_processes() {
    local signal="$1"

    pkill "-$signal" -f "$WS/install/marvin_teleop/lib/marvin_teleop/" 2>/dev/null || true
    pkill "-$signal" -f "$WS/install/marvin_ros_control/lib/marvin_ros_control/" 2>/dev/null || true
    pkill "-$signal" -f "$WS/install/node_manager/lib/node_manager/" 2>/dev/null || true
    pkill "-$signal" -f "$WS/install/bag_playback_nodes_py/lib/bag_playback_nodes_py/" 2>/dev/null || true
    pkill "-$signal" -f "$WS/install/bag_recorder_nodes_py/lib/bag_recorder_nodes_py/" 2>/dev/null || true
    pkill "-$signal" -f "$WS/install/gmsl_quadcam/lib/gmsl_quadcam/" 2>/dev/null || true
    pkill "-$signal" -f "$WS/external/realsense_runtime/lib/realsense2_camera/" 2>/dev/null || true
    pkill "-$signal" -f "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher" 2>/dev/null || true
    pkill "-$signal" -f "/opt/ros/humble/lib/rviz2/rviz2" 2>/dev/null || true
    pkill "-$signal" -f "rosbridge_websocket" 2>/dev/null || true

    # Older launch variants used these exact paths.
    pkill "-$signal" -f "$WS/install/marvin_teleop/lib/marvin_teleop/controller_udp.py" 2>/dev/null || true
}

cleanup_stale_bringup() {
    kill_bringup_processes TERM
    sleep 1
    kill_bringup_processes KILL
}

warning_light_off() {
    if [ -x "$WS/scripts/warning_light_off.py" ]; then
        "$WS/scripts/warning_light_off.py" 2>/dev/null || true
    fi
}

cleanup_bringup() {
    local status=$?

    if [ "$CLEANING_UP" -eq 1 ]; then
        exit "$status"
    fi
    CLEANING_UP=1

    echo
    echo "正在清理机器人系统残留进程..."

    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        # ros2 launch is started with setsid, so its PID is also its process group ID.
        kill -INT "-$LAUNCH_PID" 2>/dev/null || true
        sleep 2
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -TERM "-$LAUNCH_PID" 2>/dev/null || true
        fi
    fi

    sleep 1
    cleanup_stale_bringup
    warning_light_off

    echo "清理完成。"
    exit "$status"
}

trap cleanup_bringup INT TERM EXIT

# ROS 环境
source /opt/ros/humble/setup.bash

# Drop stale external RealSense workspace entries that can shadow Apex messages.
remove_path_entries_matching AMENT_PREFIX_PATH "/realsense_ws/install"
remove_path_entries_matching CMAKE_PREFIX_PATH "/realsense_ws/install"
remove_path_entries_matching LD_LIBRARY_PATH "/realsense_ws/install"
remove_path_entries_matching PYTHONPATH "/realsense_ws/install"
remove_path_entries_matching PATH "/realsense_ws/install"

# Optional vendored RealSense runtime. Do not source its generated setup files:
# they can contain absolute paths from the workspace where they were built.
REALSENSE_RUNTIME="${REALSENSE_RUNTIME:-$WS/external/realsense_runtime}"
if [ -d "$REALSENSE_RUNTIME/share/realsense2_camera" ]; then
    export AMENT_PREFIX_PATH="$REALSENSE_RUNTIME${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$REALSENSE_RUNTIME${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$REALSENSE_RUNTIME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$REALSENSE_RUNTIME/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    export PATH="$REALSENSE_RUNTIME/bin:$REALSENSE_RUNTIME/lib/realsense2_camera${PATH:+:$PATH}"
fi

# 工作区
cd "$WS"
[ -f install/setup.bash ] && source install/setup.bash
warning_light_off

if [ "${CLEAN_STALE_ON_START:-true}" = "true" ]; then
    echo "清理上次残留进程..."
    cleanup_stale_bringup
fi

echo "启动 Launch..."
setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" use_rviz:=true "$@" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
