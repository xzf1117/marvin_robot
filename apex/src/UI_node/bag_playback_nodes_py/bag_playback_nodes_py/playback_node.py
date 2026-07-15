#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import threading
import time
import json
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any


class BagPlaybackNode(Node):
    """
    ROS2 Bag回放节点
    支持播放、暂停、停止、跳转、变速等功能
    """
    
    def __init__(self):
        super().__init__('bag_playback_node')
        
        # 状态变量
        self.is_playing = False
        self.is_paused = False
        self.playback_rate = 1.0
        self.current_bag_path = None
        self.reader = None
        self.messages = []
        self.current_message_index = 0
        self.start_timestamp = None
        self.bag_duration = 0.0
        self.current_time = 0.0
        
        # 线程控制
        self.playback_thread = None
        self.stop_event = threading.Event()
        
        # 发布者字典 - 动态创建话题发布者
        self.topic_publishers = {}
        # 话题类型信息字典
        self.topic_types = {}
        
        # 订阅控制命令
        self.control_subscription = self.create_subscription(
            String,
            'control/playback_control',
            self.control_callback,
            10
        )
        
        # 发布回放状态
        self.status_publisher = self.create_publisher(
            String,
            '/playback_status',
            10
        )
        
        # 状态发布定时器
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('Bag回放节点已启动')

    def control_callback(self, msg):  #/playback_control
        """处理前端发送的控制命令"""
        try:
            command = json.loads(msg.data)
            cmd_type = command.get('type')
            
            self.get_logger().info(f'收到控制命令: {command}')
            
            if cmd_type == 'load':
                bag_path = command.get('bag_path')
                self.get_logger().info(f'尝试加载bag文件: {bag_path} (类型: {type(bag_path)})')
                self.load_bag(bag_path)
            elif cmd_type == 'play':
                self.play()
            elif cmd_type == 'pause':
                self.pause()
            elif cmd_type == 'stop':
                self.stop()
            elif cmd_type == 'seek':
                timestamp = command.get('timestamp', 0.0)
                self.seek(timestamp)
            elif cmd_type == 'set_rate':
                rate = command.get('rate', 1.0)
                self.set_playback_rate(rate)
                
        except json.JSONDecodeError:
            self.get_logger().error(f'无效的控制命令格式: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'处理控制命令时出错: {str(e)}')

    def load_bag(self, bag_path):
        """加载bag文件"""
        try:
            # 验证参数
            if not bag_path:
                self.get_logger().error('Bag文件路径不能为空')
                return False
                
            if not isinstance(bag_path, str):
                self.get_logger().error(f'Bag文件路径必须是字符串类型, 当前类型: {type(bag_path)}')
                return False
            
            if not os.path.exists(bag_path):
                self.get_logger().error(f'Bag文件不存在: {bag_path}')
                return False
            
            # 停止当前播放
            self.stop()
            
            # 初始化读取器
            self.reader = SequentialReader()
            storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            self.reader.open(storage_options, converter_options)
            
            # 读取所有消息到内存
            self.messages = []
            self.topic_publishers.clear()
            
            while self.reader.has_next():
                topic, data, timestamp = self.reader.read_next()
                self.messages.append((topic, data, timestamp))
            
            # 注意：SequentialReader在某些ROS2版本中没有close()方法
            # 对象会在离开作用域时自动清理资源
            
            if self.messages:
                self.start_timestamp = self.messages[0][2]
                end_timestamp = self.messages[-1][2]
                self.bag_duration = (end_timestamp - self.start_timestamp) / 1e9  # 转换为秒
                
                # 先设置bag路径，prepare_publishers需要用到
                self.current_bag_path = bag_path
                self.current_message_index = 0
                self.current_time = 0.0
                
                # 准备发布者（需要current_bag_path已设置）
                self.prepare_publishers()
                
                self.get_logger().info(f'成功加载bag文件: {bag_path}')
                self.get_logger().info(f'消息数量: {len(self.messages)}, 时长: {self.bag_duration:.2f}秒')
                return True
            else:
                self.get_logger().warning(f'Bag文件为空: {bag_path}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'加载bag文件失败: {str(e)}')
            return False

    def prepare_publishers(self):
        """准备话题发布者"""
        # 验证bag路径
        if not self.current_bag_path:
            self.get_logger().error('没有当前bag文件路径')
            return
        
        topic_types = {}
        
        # 分析bag中的所有话题和消息类型
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        
        reader = SequentialReader()
        storage_options = StorageOptions(uri=self.current_bag_path, storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)
        
        # 获取话题信息
        topic_metadata = reader.get_all_topics_and_types()
        self.get_logger().info(f'发现 {len(topic_metadata)} 个话题')
        
        # 为每个话题创建发布者
        for topic_info in topic_metadata:
            topic_name = topic_info.name
            topic_type = topic_info.type
            
            # 为回放话题添加"_recorded"后缀，避免与实时话题冲突
            recorded_topic = f"{topic_name}_recorded"
            
            try:
                # 动态获取消息类型
                msg_class = get_message(topic_type)
                
                # 创建QoS配置
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
                
                # 创建发布者
                self.topic_publishers[topic_name] = self.create_publisher(
                    msg_class, recorded_topic, qos
                )
                
                # 保存消息类型信息
                topic_types[topic_name] = {
                    'type': topic_type,
                    'class': msg_class,
                    'recorded_topic': recorded_topic
                }
                
                self.get_logger().info(f'创建发布者: {topic_name} ({topic_type}) -> {recorded_topic}')
                
            except Exception as e:
                self.get_logger().error(f'无法创建发布者 {topic_name} ({topic_type}): {str(e)}')
        
        # 保存话题类型信息
        self.topic_types = topic_types

    def play(self):
        """开始播放"""
        if not self.messages:
            self.get_logger().warning('没有加载bag文件')
            return
        
        if self.is_playing and not self.is_paused:
            return
        
        self.is_playing = True
        self.is_paused = False
        
        if self.playback_thread is None or not self.playback_thread.is_alive():
            self.stop_event.clear()
            self.playback_thread = threading.Thread(target=self.playback_loop)
            self.playback_thread.start()
            
        self.get_logger().info('开始播放')

    def pause(self):
        """暂停播放"""
        self.is_paused = not self.is_paused
        status = "暂停" if self.is_paused else "继续播放"
        self.get_logger().info(status)

    def stop(self):
        """停止播放"""
        self.is_playing = False
        self.is_paused = False
        self.stop_event.set()
        
        # 检查是否在当前线程内部调用stop，避免自连接
        if (self.playback_thread and 
            self.playback_thread.is_alive() and 
            threading.current_thread() != self.playback_thread):
            self.playback_thread.join(timeout=1.0)
        
        self.current_message_index = 0
        self.current_time = 0.0
        self.get_logger().info('停止播放')

    def seek(self, target_time):
        """跳转到指定时间（秒）"""
        if not self.messages:
            return
        
        if self.start_timestamp is not None:
            target_timestamp = self.start_timestamp + int(target_time * 1e9)
        else:
            return
        
        # 查找最接近的消息索引
        for i, (topic, data, timestamp) in enumerate(self.messages):
            if timestamp >= target_timestamp:
                self.current_message_index = i
                self.current_time = target_time
                break
        
        self.get_logger().info(f'跳转到时间: {target_time:.2f}秒')

    def set_playback_rate(self, rate):
        """设置播放速度"""
        self.playback_rate = max(0.1, min(rate, 5.0))  # 限制在0.1x到5x之间
        self.get_logger().info(f'设置播放速度: {self.playback_rate}x')

    def playback_loop(self):
        """播放循环（在单独线程中运行）"""
        # 记录播放开始的实际时间和bag时间
        playback_start_time = time.time()
        bag_start_time = self.start_timestamp
        
        # 如果从中间开始播放，调整开始时间
        if self.current_message_index > 0:
            current_msg_timestamp = self.messages[self.current_message_index][2]
            bag_start_time = current_msg_timestamp
            self.current_time = (current_msg_timestamp - self.start_timestamp) / 1e9
        
        self.get_logger().info(f'开始播放，从索引 {self.current_message_index} 开始，当前时间: {self.current_time:.3f}s')
        
        while (self.is_playing and 
               self.current_message_index < len(self.messages) and 
               not self.stop_event.is_set()):
            
            if self.is_paused:
                time.sleep(0.01)
                # 暂停时重新记录起始时间，以便恢复后时间连续
                playback_start_time = time.time()
                if self.current_message_index < len(self.messages):
                    bag_start_time = self.messages[self.current_message_index][2]
                continue
            
            # 获取当前消息
            topic, data, timestamp = self.messages[self.current_message_index]
            
            # 计算消息在bag中的相对时间（从当前播放位置开始）
            msg_relative_time = (timestamp - bag_start_time) / 1e9
            
            # 计算实际应该播放的时间点（考虑播放速度）
            target_playback_time = playback_start_time + (msg_relative_time / self.playback_rate)
            
            # 当前实际时间
            current_real_time = time.time()
            
            # 如果还没到播放时间，等待
            if current_real_time < target_playback_time:
                sleep_time = target_playback_time - current_real_time
                if sleep_time > 0.001:  # 如果等待时间太短，就不sleep了
                    time.sleep(min(sleep_time, 0.1))  # 最多sleep 100ms
                continue
            
            # 时间到了，发布消息
            self.publish_message(topic, data)
            self.current_time = (timestamp - self.start_timestamp) / 1e9
            self.current_message_index += 1
            
            # 调试信息（可选）
            if self.current_message_index % 10 == 0:  # 每10条消息打印一次
                self.get_logger().debug(f'播放进度: {self.current_message_index}/{len(self.messages)}, 时间: {self.current_time:.3f}s')
            
        # 播放完成
        if self.current_message_index >= len(self.messages):
            self.get_logger().info('播放完成')
            # 只设置播放状态，避免在线程内部调用stop()导致自连接错误
            self.is_playing = False
            self.is_paused = False
            self.current_message_index = 0
            self.current_time = 0.0
            # 发送播放完成的状态更新
            self.publish_status()
            return  # 退出循环，让线程自然结束

    def publish_message(self, topic, data):
        """发布消息到对应话题"""
        try:
            if topic in self.topic_publishers and topic in self.topic_types:
                # 获取消息类型
                msg_class = self.topic_types[topic]['class']
                recorded_topic = self.topic_types[topic]['recorded_topic']
                
                # 反序列化消息
                msg = deserialize_message(data, msg_class)
                
                # 发布消息
                self.topic_publishers[topic].publish(msg)
                
                # 调试信息（可选，避免过多日志）
                # self.get_logger().debug(f'发布消息到 {recorded_topic}')
                
        except Exception as e:
            if hasattr(self, 'topic_types') and topic in self.topic_types:
                topic_type = self.topic_types[topic]['type']
                self.get_logger().error(f'发布消息失败 [{topic}] ({topic_type}): {str(e)}')
            else:
                self.get_logger().error(f'发布消息失败 [{topic}]: {str(e)}')

    def publish_status(self):
        """发布回放状态"""
        status = {
            'is_playing': self.is_playing,
            'is_paused': self.is_paused,
            'playback_rate': self.playback_rate,
            'current_time': self.current_time,
            'duration': self.bag_duration,
            'progress': self.current_time / self.bag_duration if self.bag_duration > 0 else 0.0,
            'bag_path': self.current_bag_path
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BagPlaybackNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 