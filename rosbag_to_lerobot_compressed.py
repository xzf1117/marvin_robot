#!/usr/bin/env python3
import os
import glob
import numpy as np
import cv2
from pathlib import Path

# 引入纯 Python 的 MCAP 解包器
from mcap_ros2.reader import read_ros2_messages
# 引入 LeRobot 3.0 数据集核心类
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs.video import RGBEncoderConfig

# ---------------------------------------------------------
# 🛠️ 配置区域
# ---------------------------------------------------------
# 输入包含 rosbag2_xxx 文件夹的绝对路径
BAGS_DIR = os.path.expanduser("/media/zju/PortableSSD/robot_data") 
# 输出 LeRobot 数据集的仓库 ID（保存在 ~/.cache/huggingface/lerobot 或自定义位置）
REPO_ID = "zhifengxu/marvin_pick_and_place_v1"
TASK_PROMPT = "Pick up the block and place it into the container"

# 时间戳对齐阈值 (单位: 纳秒) -> 33ms (约 1 帧的允许抖动窗口)
TIMESTAMP_THRESHOLD_NS = 33_000_000 

# ---------------------------------------------------------
# 📹 视频编码器设置
# ---------------------------------------------------------
rgb_encoder = RGBEncoderConfig(
    vcodec="h264",       # H.264 硬件/软件解码友好
    g=2,                 # GOP=2 训练读取与随机抽帧优化
    crf=18,              # CRF=18 视觉无损质量
    pix_fmt="yuv420p",
    fast_decode=1,
)

# ---------------------------------------------------------
# 📦 创建标准的 LeRobotV3 数据集结构
# ---------------------------------------------------------
cam_head_h = 720
cam_head_w = 1280
cam_wrist_h = 480
cam_wrist_w = 848

dataset = LeRobotDataset.create(
    repo_id=REPO_ID,
    fps=30,
    features={
        "observation.images.cam_head": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.cam_left_wrist": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.cam_right_wrist": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (16,),  # 14 关节角 + 2 夹爪反馈
        },
        "action": {
            "dtype": "float32",
            "shape": (16,),  # 下一时刻的目标 state
        },
    },
    use_videos=True,
    robot_type="marvin",
    rgb_encoder=rgb_encoder,
)

# ---------------------------------------------------------
# 🔍 图像解码辅助函数
# ---------------------------------------------------------
def decode_compressed_image(msg, target_size=(640, 480)):
    """纯 Python 解析 ROS 2 CompressedImage (JPEG/PNG) 字节流，转为 RGB"""
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None:
            return None
        
        # LeRobot 内部统一采用 RGB 格式
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        # 若需要无损中心裁剪 (例如 848x480 -> 640x480)
        h, w, _ = img_rgb.shape
        if w == cam_wrist_w and h == cam_wrist_h:
            start_x = (cam_wrist_w - target_size[0]) // 2  # (848 - 640) // 2 = 104
            img_rgb = img_rgb[:, start_x:start_x + target_size[0], :]
        elif w == cam_head_w and h == cam_head_h:
            img_rgb = cv2.resize(img_rgb, target_size, interpolation=cv2.INTER_AREA)
        elif w != target_size[0] or h != target_size[1]:
            img_rgb = cv2.resize(img_rgb, target_size, interpolation=cv2.INTER_AREA)    
        
        return img_rgb
    except Exception as e:
        print(f"  ❌ 图像解码异常: {e}")
        return None

# ---------------------------------------------------------
# 🚀 核心逻辑：解析单个 MCAP 并打包为 Episode
# ---------------------------------------------------------
def convert_bag_to_episode(mcap_file_path, episode_idx):
    print(f"\n🔄 [{episode_idx}] 离线解析文件: {mcap_file_path}")
    
    head_images = []
    left_wrist_images = []
    right_wrist_images = []
    joint_states = []
    gripper_L = []
    gripper_R = []

    # 1. 遍历 MCAP 文件提取所有消息
    for message in read_ros2_messages(mcap_file_path):
        topic = message.channel.topic
        timestamp = message.publish_time_ns
        msg = message.ros_msg

        # 相机图像分支 (针对 compressed 话题)
        if topic == "/camera/cam_head/color/image_raw/compressed":
            img = decode_compressed_image(msg)
            if img is not None:
                head_images.append((timestamp, img))
                
        elif topic == "/camera/cam_left_wrist/color/image_raw/compressed":
            img = decode_compressed_image(msg)
            if img is not None:
                left_wrist_images.append((timestamp, img))
                
        elif topic == "/camera/cam_right_wrist/color/image_raw/compressed":
            img = decode_compressed_image(msg)
            if img is not None:
                right_wrist_images.append((timestamp, img))

        # 状态与夹爪反馈分支
        elif topic == "/joint_states":
            if hasattr(msg, 'position') and len(msg.position) >= 14:
                joint_states.append((timestamp, np.array(msg.position[:14], dtype=np.float32)))
                
        elif topic == "/info/gripper_feedback_L":
            if hasattr(msg, 'data') and len(msg.data) > 0:
                gripper_L.append((timestamp, msg.data[0]))
                
        elif topic == "/info/gripper_feedback_R":
            if hasattr(msg, 'data') and len(msg.data) > 0:
                gripper_R.append((timestamp, msg.data[0]))

    # 数据完整性校验
    if not (head_images and left_wrist_images and right_wrist_images and joint_states and gripper_L and gripper_R):
        print(f"⚠️ 警告: 话题数据不全，跳过当前文件。")
        print(f"   相机帧数: head={len(head_images)}, left={len(left_wrist_images)}, right={len(right_wrist_images)}")
        print(f"   状态帧数: joints={len(joint_states)}, gL={len(gripper_L)}, gR={len(gripper_R)}")
        return

    # 2. 以 cam_head 时间戳为主轴进行多通道 Nearest-Neighbor 软对齐
    aligned_frames = []
    for t_head, img_head in head_images:
        idx_lw = np.argmin([abs(t - t_head) for t, _ in left_wrist_images])
        idx_rw = np.argmin([abs(t - t_head) for t, _ in right_wrist_images])
        idx_js = np.argmin([abs(t - t_head) for t, _ in joint_states])
        idx_gL = np.argmin([abs(t - t_head) for t, _ in gripper_L])
        idx_gR = np.argmin([abs(t - t_head) for t, _ in gripper_R])

        # 超出时间阈值的帧判定为断流，丢弃该帧以保证准确性
        if abs(left_wrist_images[idx_lw][0] - t_head) > TIMESTAMP_THRESHOLD_NS or \
           abs(right_wrist_images[idx_rw][0] - t_head) > TIMESTAMP_THRESHOLD_NS or \
           abs(joint_states[idx_js][0] - t_head) > TIMESTAMP_THRESHOLD_NS:
            continue

        # 组装 16 维 state向量 (14 关节 + 2 夹爪)
        state_16d = np.zeros(16, dtype=np.float32)
        state_16d[:14] = joint_states[idx_js][1]
        state_16d[14] = gripper_L[idx_gL][1]
        state_16d[15] = gripper_R[idx_gR][1]

        aligned_frames.append({
            "img_head": img_head,
            "img_lw": left_wrist_images[idx_lw][1],
            "img_rw": right_wrist_images[idx_rw][1],
            "state": state_16d
        })

    total_aligned = len(aligned_frames)
    if total_aligned < 200:
        print("⚠️ 警告: 对齐后帧数过少，跳过此 Episode。")
        return

    # 3. 构造 Action (下一帧的 state) 并追加到 LeRobot Dataset
    for i in range(total_aligned - 1):
        current_data = aligned_frames[i]
        next_data = aligned_frames[i + 1]

        dataset.add_frame({
            "observation.images.cam_head": current_data["img_head"],
            "observation.images.cam_left_wrist": current_data["img_lw"],
            "observation.images.cam_right_wrist": current_data["img_rw"],
            "observation.state": current_data["state"],
            "action": next_data["state"],  # 模仿学习标准定义：action = next_state
            "task": TASK_PROMPT,
        })

    dataset.save_episode()
    print(f"✅ 第 {episode_idx} 组 Episode 转换完毕，成功写入 {total_aligned - 1} 帧！")

# ---------------------------------------------------------
# 🏁 主入口
# ---------------------------------------------------------
if __name__ == "__main__":
    # 递归查找数据文件夹下所有的 .mcap 文件
    mcap_files = sorted(glob.glob(os.path.join(BAGS_DIR, "**/rosbag2_*.mcap"), recursive=True))
    print(f"📋 共扫描到 {len(mcap_files)} 个 .mcap 文件，开始离线打包转换...")

    for idx, filepath in enumerate(mcap_files):
        convert_bag_to_episode(filepath, episode_idx=idx)

    # 终结并生成数据集 index/meta 信息
    dataset.finalize()
    print("\n🎉 全流程离线转换完成！数据集已成功生成并保存。")