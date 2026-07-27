#!/usr/bin/env python3
import os
import glob
import numpy as np
import cv2
import shutil
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

from mcap_ros2.reader import read_ros2_messages
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs.video import RGBEncoderConfig

BAGS_DIR = os.path.expanduser("/media/zju/PortableSSD/robot_data") 
REPO_ID = "zhifengxu/marvin_pick_and_place_v1"
TASK_PROMPT = "Pick up the block and place it into the container"

TIMESTAMP_THRESHOLD_NS = 33_000_000 
append_dataset = True  # 是否在已有数据集上追加新 Episode，False 则清理旧缓存

# ⚡ 优化6: crf=23, g=12 — Diffusion Policy / Pi0.5 完全够用，编码速度大幅提升
rgb_encoder = RGBEncoderConfig(
    vcodec="h264",
    g=12,
    crf=23,
    pix_fmt="yuv420p",
    fast_decode=1,
)

cam_head_h = 720
cam_head_w = 1280
cam_wrist_h = 480
cam_wrist_w = 848

dataset_cache_dir = Path.home() / ".cache/huggingface/lerobot" / REPO_ID

if dataset_cache_dir.exists():
    if not append_dataset:
        print(f"🧹 发现未完成或损坏的数据集缓存 [{REPO_ID}]，正在清理旧数据...")
        shutil.rmtree(dataset_cache_dir)
    else:
        raise RuntimeError(
        f"❌ 数据集缓存已存在 [{REPO_ID}]，位于 {dataset_cache_dir}\n"
    )
    
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
            "shape": (16,),
        },
        "action": {
            "dtype": "float32",
            "shape": (16,),
        },
    },
    use_videos=True,
    robot_type="marvin",
    rgb_encoder=rgb_encoder,
)

# ⚡ 优化2: decode_compressed_image 接收 raw bytes 而非完整 msg 对象
def decode_compressed_image(data: bytes, target_size=(640, 480)):
    try:
        np_arr = np.frombuffer(data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None:
            return None
        
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = img_rgb.shape
        
        if w == cam_wrist_w and h == cam_wrist_h:
            start_x = (cam_wrist_w - target_size[0]) // 2
            img_rgb = img_rgb[:, start_x:start_x + target_size[0], :]
        elif w == cam_head_w and h == cam_head_h:
            # ⚡ 优化5: INTER_LINEAR — 相比 INTER_AREA 速度快很多，质量损失可忽略
            img_rgb = cv2.resize(img_rgb, target_size, interpolation=cv2.INTER_LINEAR)
        elif w != target_size[0] or h != target_size[1]:
            img_rgb = cv2.resize(img_rgb, target_size, interpolation=cv2.INTER_LINEAR)    
        
        return img_rgb
    except Exception as e:
        print(f"  ❌ 图像解码异常: {e}")
        return None

def find_nearest_indices(target_ts, source_ts):
    """⚡ 使用 NumPy 二分查找 (np.searchsorted) 秒级计算最近邻索引"""
    idx = np.searchsorted(source_ts, target_ts, side="left")
    idx = np.clip(idx, 1, len(source_ts) - 1)
    
    # 比较前后两个相邻时间戳，选较近的那一个
    left_diff = np.abs(source_ts[idx - 1] - target_ts)
    right_diff = np.abs(source_ts[idx] - target_ts)
    
    return np.where(left_diff < right_diff, idx - 1, idx)

# ⚡ 优化3: 全局线程池，各 episode 复用，cv2.imdecode 释放 GIL 实现真实并行
_executor = ThreadPoolExecutor(max_workers=4)

def convert_bag_to_episode(mcap_file_path, episode_idx):
    print(f"\n🔄 [{episode_idx}] 离线解析文件: {mcap_file_path}")
    
    # ⚡ 1. 阶段一：只存原始字节和时间戳
    #    ⚡ 优化1: 存 bytes(msg.data) 而非完整 msg 对象，大幅省内存+提升 cache 命中率
    head_raw = []
    left_wrist_raw = []
    right_wrist_raw = []
    joint_states = []
    gripper_L = []
    gripper_R = []

    for message in read_ros2_messages(mcap_file_path):
        topic = message.channel.topic
        timestamp = message.publish_time_ns
        msg = message.ros_msg

        # 过滤只关心的 7 个话题，只保存原生 raw bytes/msg
        if topic == "/camera/cam_head/color/image_raw/compressed":
            head_raw.append((timestamp, bytes(msg.data)))
        elif topic == "/camera/cam_left_wrist/color/image_raw/compressed":
            left_wrist_raw.append((timestamp, bytes(msg.data)))
        elif topic == "/camera/cam_right_wrist/color/image_raw/compressed":
            right_wrist_raw.append((timestamp, bytes(msg.data)))
        elif topic == "/joint_states":
            if hasattr(msg, 'position') and len(msg.position) >= 14:
                joint_states.append((timestamp, np.array(msg.position[:14], dtype=np.float32)))
        elif topic == "/info/gripper_feedback_L":
            if hasattr(msg, 'data') and len(msg.data) > 0:
                gripper_L.append((timestamp, float(msg.data[0])))
        elif topic == "/info/gripper_feedback_R":
            if hasattr(msg, 'data') and len(msg.data) > 0:
                gripper_R.append((timestamp, float(msg.data[0])))

    # 校验基础数据完整性
    if not (head_raw and left_wrist_raw and right_wrist_raw and joint_states and gripper_L and gripper_R):
        print(f"⚠️ 警告: 话题数据不全，跳过当前文件。")
        return

    # ⚡ 2. 阶段二：使用 NumPy 毫秒级对齐时间戳 (纯数字运算，不涉及图像)
    t_head = np.array([t for t, _ in head_raw], dtype=np.int64)
    t_lw = np.array([t for t, _ in left_wrist_raw], dtype=np.int64)
    t_rw = np.array([t for t, _ in right_wrist_raw], dtype=np.int64)
    t_js = np.array([t for t, _ in joint_states], dtype=np.int64)
    t_gL = np.array([t for t, _ in gripper_L], dtype=np.int64)
    t_gR = np.array([t for t, _ in gripper_R], dtype=np.int64)

    indices_lw = find_nearest_indices(t_head, t_lw)
    indices_rw = find_nearest_indices(t_head, t_rw)
    indices_js = find_nearest_indices(t_head, t_js)
    indices_gL = find_nearest_indices(t_head, t_gL)
    indices_gR = find_nearest_indices(t_head, t_gR)

    aligned_indices = []
    for i in range(len(head_raw)):
        idx_lw = indices_lw[i]
        idx_rw = indices_rw[i]
        idx_js = indices_js[i]

        # 阈值过滤
        if abs(t_lw[idx_lw] - t_head[i]) > TIMESTAMP_THRESHOLD_NS or \
           abs(t_rw[idx_rw] - t_head[i]) > TIMESTAMP_THRESHOLD_NS or \
           abs(t_js[idx_js] - t_head[i]) > TIMESTAMP_THRESHOLD_NS:
            continue

        aligned_indices.append((i, idx_lw, idx_rw, idx_js, indices_gL[i], indices_gR[i]))

    if len(aligned_indices) < 200:
        print("⚠️ 警告: 对齐后帧数过少，跳过此 Episode。")
        return

    # ⚡ 阶段三：流式解码 + add_frame（无 aligned_frames 缓存）
    # ⚡ 优化3+4: 用 ThreadPoolExecutor 并行解码三个 camera，边 decode 边 add_frame 边释放
    total_aligned = 0
    
    prev_state = None
    prev_img_head = None
    prev_img_lw = None
    prev_img_rw = None
    first_frame = True

    for idx_h, idx_lw, idx_rw, idx_js, idx_gL, idx_gR in aligned_indices:
        # 三个 camera 并行 JPEG 解码
        f_head = _executor.submit(decode_compressed_image, head_raw[idx_h][1])
        f_lw   = _executor.submit(decode_compressed_image, left_wrist_raw[idx_lw][1])
        f_rw   = _executor.submit(decode_compressed_image, right_wrist_raw[idx_rw][1])
        
        img_head = f_head.result()
        img_lw   = f_lw.result()
        img_rw   = f_rw.result()

        if img_head is None or img_lw is None or img_rw is None:
            first_frame = True  # 解码失败则重置，不跨 gap 连接
            continue

        state_16d = np.zeros(16, dtype=np.float32)
        state_16d[:14] = joint_states[idx_js][1]
        state_16d[14] = gripper_L[idx_gL][1]
        state_16d[15] = gripper_R[idx_gR][1]

        if first_frame:
            # 缓存第一帧，等待下一帧作为 action
            first_frame = False
            prev_state = state_16d
            prev_img_head = img_head
            prev_img_lw = img_lw
            prev_img_rw = img_rw
            total_aligned += 1
            continue

        # 使用 prev_xxx 作为 observation，当前帧 state 作为 action
        dataset.add_frame({
            "observation.images.cam_head": prev_img_head,
            "observation.images.cam_left_wrist": prev_img_lw,
            "observation.images.cam_right_wrist": prev_img_rw,
            "observation.state": prev_state,
            "action": state_16d,
            "task": TASK_PROMPT,
        })

        # 滚动：当前帧 → 上一帧，当前图片立即丢弃（del 确保尽早释放内存）
        prev_state = state_16d
        prev_img_head = img_head
        prev_img_lw = img_lw
        prev_img_rw = img_rw
        total_aligned += 1

        # 显式删除旧图片引用，让 GC 尽早回收
        # Python 引用计数会在下次循环覆盖前自动回收，这里不需要显式 del

    dataset.save_episode()
    print(f"✅ 第 {episode_idx} 组 Episode 转换完毕，成功写入 {total_aligned - 1} 帧！")

if __name__ == "__main__":
    mcap_files = sorted(glob.glob(os.path.join(BAGS_DIR, "**/rosbag2_20260723*.mcap"), recursive=True))
    print(f"📋 共扫描到 {len(mcap_files)} 个 .mcap 文件，开始离线打包转换...")

    for idx, filepath in enumerate(mcap_files):
        convert_bag_to_episode(filepath, episode_idx=idx)

    _executor.shutdown(wait=False)
    dataset.finalize()
    print("\n🎉 全流程离线转换完成！数据集已成功生成并保存。")