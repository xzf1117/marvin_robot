import os
import glob
import numpy as np
import cv2
from io import BytesIO
from PIL import Image as PILImage

# 引入纯 Python 的 MCAP 解包器
from mcap_ros2.reader import read_ros2_messages
# 引入 LeRobot 3.0 数据集核心类
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs.video import RGBEncoderConfig

BAGS_DIR = os.path.expanduser("/opt/kernelmind/") 
REPO_ID = "zhifengxu/marvin_bimanual_v3"

rgb_encoder = RGBEncoderConfig(
    vcodec="h264",       # 压制 H.264，4090 训练解码极快
    g=2,                 # GOP=2 训练读取神级优化
    crf=18,              # CRF=18 视觉无损，保留一切方块和指尖边缘
    pix_fmt="yuv420p",
    fast_decode=1,
)

# 创建标准的 LeRobotV3 数据集
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

def decode_ros_image(msg):
    """纯 Python 解析 ROS 2 Image 字节流，完美绕过 cv_bridge"""
    # msg.encoding 可能是 'rgb8' 或 'bgr8'
    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if "bgr" in msg.encoding.lower():
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2RGB)
    
    h, w, c = img_data.shape
    if w == 848 and h == 480:
        # 计算左右裁剪的边界
        start_x = (848 - 640) // 2  # 104 像素
        end_x = start_x + 640        # 744 像素
        img_data = img_data[:, start_x:end_x, :]  # 完美无损中心裁剪
        
    return img_data

def convert_bag_to_episode(mcap_file_path, episode_idx):
    print(f"🔄 正在通过纯 Python 引擎离线解析: {mcap_file_path}")
    
    head_images = []
    left_wrist_images = []
    right_wrist_images = []
    joint_states = []
    gripper_L = []
    gripper_R = []

    # 纯 Python 读取 MCAP，不需要 source ROS2 环境变量，不依赖 C++ 动态链接库！
    for message in read_ros2_messages(mcap_file_path):
        topic = message.channel.topic
        timestamp = message.publish_time_ns
        msg = message.ros_msg

        if topic == "/camera/cam_head_launch/color/image_raw":
            head_images.append((timestamp, decode_ros_image(msg)))
        elif topic == "/camera/cam_left_wrist/color/image_raw":
            left_wrist_images.append((timestamp, decode_ros_image(msg)))
        elif topic == "/camera/cam_right_wrist/color/image_raw":
            right_wrist_images.append((timestamp, decode_ros_image(msg)))
        
        # if topic == "/camera/cam_head_launch/color/image_raw/compressed":
        #     head_images.append((timestamp, decode_ros_image(msg)))
        # elif topic == "/camera/cam_left_wrist/color/image_raw/compressed":
        #     left_wrist_images.append((timestamp, decode_ros_image(msg)))
        # elif topic == "/camera/cam_right_wrist/color/image_raw/compressed":
        #     right_wrist_images.append((timestamp, decode_ros_image(msg)))

        elif topic == "/joint_states":
            joint_states.append((timestamp, np.array(msg.position[:14], dtype=np.float32)))
        elif topic == "/info/gripper_feedback_L":
            gripper_L.append((timestamp, msg.data[0]))
        elif topic == "/info/gripper_feedback_R":
            gripper_R.append((timestamp, msg.data[0]))

    if len(head_images) == 0:
        print(f"⚠️ 警告: 未找到头部相机画面，跳过。")
        return

    aligned_frames = []
    for t_head, img_head in head_images:
        idx_lw = np.argmin([abs(t - t_head) for t, _ in left_wrist_images]) if left_wrist_images else None
        idx_rw = np.argmin([abs(t - t_head) for t, _ in right_wrist_images]) if right_wrist_images else None
        idx_js = np.argmin([abs(t - t_head) for t, _ in joint_states]) if joint_states else None
        idx_gL = np.argmin([abs(t - t_head) for t, _ in gripper_L]) if gripper_L else None
        idx_gR = np.argmin([abs(t - t_head) for t, _ in gripper_R]) if gripper_R else None
        
        # 16ms 内对齐判定
        if idx_lw is None or abs(left_wrist_images[idx_lw][0] - t_head) > 16_000_000:
            continue
            
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

    # 生成 Action 并写入 LeRobot
    total_aligned = len(aligned_frames)
    task_prompt = "Pick up the block and place it into the container"
    for i in range(total_aligned - 1):
        current_data = aligned_frames[i]
        next_data = aligned_frames[i + 1]
        
        dataset.add_frame({
            "observation.images.cam_head": current_data["img_head"],
            "observation.images.cam_left_wrist": current_data["img_lw"],
            "observation.images.cam_right_wrist": current_data["img_rw"],
            "observation.state": current_data["state"],
            "action": next_data["state"],
            "task": task_prompt,
        })
        
    dataset.save_episode()
    print(f"✅ 第 {episode_idx} 组 Episode 转换完毕，包含 {total_aligned - 1} 帧。")

if __name__ == "__main__":
    # 递归查找你的所有 mcap 单文件
    mcap_files = sorted(glob.glob(os.path.join(BAGS_DIR, "**/rosbag2_*.mcap"), recursive=True))

    for idx, filepath in enumerate(mcap_files):
        convert_bag_to_episode(filepath, episode_idx=idx)
        
    dataset.finalize()
    print("🎉 纯 Python 引擎全流程离线转换成功！")