#!/usr/bin/env python3
import os
import shutil
import subprocess
import re
import json
from pathlib import Path

# 对应你要检查的数据目录
DATA_DIR = Path("/media/zju/PortableSSD/robot_data")

# 🚨 容忍阈值配置：
# 例如：要求三路相机的最大帧数与最小帧数相差不能超过 20%
MAX_FRAME_DIFF_RATIO = 0.25 

TOPIC_CHECK_MAP = {
    'cam_head': '/camera/cam_head/color/image_raw/compressed',
    'cam_left_wrist': '/camera/cam_left_wrist/color/image_raw/compressed',
    'cam_right_wrist': '/camera/cam_right_wrist/color/image_raw/compressed',
    'joint_states': '/joint_states',
    'arm_state': '/info/arm_state',
    'gripper_feedback_R': '/info/gripper_feedback_R',
    'gripper_feedback_L': '/info/gripper_feedback_L',
}

def topic_check(bag_path):
    """通过 ros2 bag info 提取话题数据帧数"""
    try:
        cmd = ["ros2", "bag", "info", str(bag_path)]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        counts = {}
        pattern = re.compile(r'Topic:\s*([^\s|]+).*?Count:\s*(\d+)')
        for line in result.stdout.splitlines():
            match = pattern.search(line)
            if match:
                topic_name = match.group(1).strip()
                count = int(match.group(2))
                counts[topic_name] = count
        return counts
    except Exception as e:
        print(f"  ❌ 解析失败 {bag_path.name}: {e}")
        return None

def main():
    if not DATA_DIR.exists():
        print(f"❌ 错误: 路径不存在 {DATA_DIR}")
        return
    
    bag_dirs = sorted([p for p in DATA_DIR.iterdir() if p.is_dir() and p.name.startswith("rosbag2_")])
    print(f"🔍 找到 {len(bag_dirs)} 个 rosbag 文件夹，开始检测...\n")

    deleted_count = 0
    kept_count = 0

    for bag_dir in bag_dirs:
        counts = topic_check(bag_dir)
        
        if counts is None:
            print(f"🗑️ [删除] {bag_dir.name} -> 无法读取包信息/数据损坏")
            shutil.rmtree(bag_dir)
            deleted_count += 1
            continue

        missing_topics = []
        parsed_topics_info = {}

        for alias, full_topic_name in TOPIC_CHECK_MAP.items():
            count = counts.get(full_topic_name, 0)
            if count <= 0:
                missing_topics.append(alias)
            parsed_topics_info[alias] = count

        if missing_topics:
            print(f"🗑️ [删除] {bag_dir.name} -> 话题不全或帧数为0! 缺失话题: {missing_topics}")
            shutil.rmtree(bag_dir)
            deleted_count += 1
            continue

        cam_counts = [
            parsed_topics_info['cam_head'],
            parsed_topics_info['cam_left_wrist'],
            parsed_topics_info['cam_right_wrist']
        ]

        max_c = max(cam_counts)
        min_c = min(cam_counts)

        # 计算极差比例
        diff_ratio = (max_c - min_c) / max_c if max_c > 0 else 1.0

        if diff_ratio > MAX_FRAME_DIFF_RATIO:
            print(f"🗑️ [删除] {bag_dir.name} -> 相机帧数差距过大! Head:{cam_counts[0]}, Left:{cam_counts[1]}, Right:{cam_counts[2]} (极差: {diff_ratio*100:.1f}%)")
            shutil.rmtree(bag_dir)
            deleted_count += 1
        else:
            print(f"✅ [保留] {bag_dir.name} -> 校验通过 | 相机帧数(H/L/R): {cam_counts} | 控制帧数: {parsed_topics_info['joint_states']}")
            kept_count += 1

    print("\n" + "="*40)
    print(f"🎉 清理完成！保留: {kept_count} 组 | 删除: {deleted_count} 组")

if __name__ == "__main__":
    main()