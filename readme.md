# 遥操作使用方法
```zsh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link add dev vcan1 type vcan 2>/dev/null || true
sudo ip link set up vcan0
sudo ip link set up vcan1
ip link show | grep -E "vcan|can"
cd /opt/kernelmind/apex 
source install/setup.zsh
./bringup_RM.sh use_rviz:=false
```
# 数据录制
```zsh
ros2 bag record \                               
  /joint_states /info/arm_state /info/gripper_feedback_L /info/gripper_feedback_R \
  /camera/cam_left_wrist/color/image_raw/compressed \
  /camera/cam_right_wrist/color/image_raw/compressed \
  /camera/cam_head/color/image_raw/compressed \
  -s mcap \
  --max-cache-size 1073741824 \
  --max-bag-size 0 \
  -o /media/zju/PortableSSD/robot_data/rosbag2_$(date +%Y%m%d_%H%M%S)
```