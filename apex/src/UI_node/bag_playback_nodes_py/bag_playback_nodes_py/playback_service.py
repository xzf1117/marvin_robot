#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import threading
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import time
from typing import Dict, List, Any


class PlaybackHTTPHandler(BaseHTTPRequestHandler):
    """HTTP请求处理器"""
    
    def __init__(self, playback_service, *args, **kwargs):
        self.playback_service = playback_service
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """处理GET请求"""
        try:
            parsed_path = urlparse(self.path)
            path = parsed_path.path
            
            if path == '/api/bag_files':
                self.handle_get_bag_files()
            elif path == '/api/bag_info':
                query = parse_qs(parsed_path.query)
                bag_path = query.get('path', [''])[0]
                self.handle_get_bag_info(bag_path)
            elif path == '/api/playback_status':
                self.handle_get_status()
            else:
                self.send_error(404, 'Not Found')
                
        except Exception as e:
            self.send_error(500, str(e))
    
    def do_POST(self):
        """处理POST请求"""
        try:
            parsed_path = urlparse(self.path)
            path = parsed_path.path
            
            # 读取请求体
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length).decode('utf-8')
            
            if path == '/api/playback_control':
                data = json.loads(post_data) if post_data else {}
                self.handle_playback_control(data)
            else:
                self.send_error(404, 'Not Found')
                
        except Exception as e:
            self.send_error(500, str(e))
    
    def handle_get_bag_files(self):
        """获取bag文件列表"""
        bag_files = self.playback_service.get_bag_files()
        self.send_json_response(bag_files)
    
    def handle_get_bag_info(self, bag_path):
        """获取bag文件信息"""
        if not bag_path:
            self.send_error(400, 'Missing bag_path parameter')
            return
        
        # 解析查询参数
        parsed_path = urlparse(self.path)
        query_params = parse_qs(parsed_path.query)
        fast_mode = query_params.get('fast_mode', ['true'])[0].lower() in ['true', '1', 'yes']
        
        bag_info = self.playback_service.get_bag_info(bag_path, fast_mode=fast_mode)
        if bag_info:
            self.send_json_response(bag_info)
        else:
            self.send_error(404, 'Bag file not found')
    
    def handle_get_status(self):
        """获取回放状态"""
        status = self.playback_service.get_playback_status()
        self.send_json_response(status)
    
    def handle_playback_control(self, data):
        """处理回放控制命令"""
        result = self.playback_service.control_playback(data)
        self.send_json_response({'success': result})
    
    def send_json_response(self, data):
        """发送JSON响应"""
        response = json.dumps(data, ensure_ascii=False)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(response.encode('utf-8'))
    
    def do_OPTIONS(self):
        """处理OPTIONS请求（CORS预检）"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        """重写日志方法，使用ROS日志"""
        self.playback_service.get_logger().debug(f"HTTP: {format % args}")


class PlaybackService(Node):
    """
    回放服务节点
    提供HTTP接口给前端，管理回放控制
    """
    
    def __init__(self):
        super().__init__('playback_service')
        
        # 配置
        self.bag_directory = str(self.declare_parameter('bag_directory', '/home/hanwen/pyq_files/UI/bags_convert/ros2bags').value)
        self.http_port = int(self.declare_parameter('http_port', 8081).value)
        
        # ROS通信
        self.control_publisher = self.create_publisher(String, 'control/playback_control', 10)
        self.status_subscriber = self.create_subscription(
            String, '/playback_status', self.status_callback, 10
        )
        
        # 状态管理
        self.current_status = {
            'is_playing': False,
            'is_paused': False,
            'playback_rate': 1.0,
            'current_time': 0.0,
            'duration': 0.0,
            'progress': 0.0,
            'bag_path': None
        }
        
        # 🚀 新增：bag详细信息缓存
        self.bag_info_cache = {}
        
        # 启动HTTP服务器
        self.start_http_server()
        
        self.get_logger().info(f'Playback service started, HTTP port: {self.http_port}')
        self.get_logger().info(f'Bag directory: {self.bag_directory}')
    
    def status_callback(self, msg):
        """接收回放状态更新"""
        try:
            self.current_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid status message format: {msg.data}')
    
    def start_http_server(self):
        """启动HTTP服务器"""
        def handler(*args, **kwargs):
            return PlaybackHTTPHandler(self, *args, **kwargs)
        
        self.httpd = HTTPServer(('0.0.0.0', self.http_port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever)
        self.http_thread.daemon = True
        self.http_thread.start()
    
    def get_bag_files(self) -> List[Dict[str, Any]]:
        """获取bag文件列表（快速扫描，只返回基本信息）"""
        bag_files = []
        
        try:
            # 搜索bag文件目录
            bag_dir = Path(self.bag_directory)
            
            # 查找所有包含metadata.yaml的目录（ROS2 bag格式）
            for item in bag_dir.rglob('*'):
                if item.is_dir() and (item / 'metadata.yaml').exists():
                    try:
                        # 只获取基本文件系统信息（快速）
                        stat_info = item.stat()
                        db_files = list(item.glob('*.db3'))
                        total_size = sum(f.stat().st_size for f in db_files)
                        
                        bag_info = {
                            'path': str(item),
                            'name': item.name,
                            'size': total_size,
                            'modified_time': stat_info.st_mtime,
                            'created_time': stat_info.st_ctime,
                            # 标记详细信息未加载
                            'detailed_info_loaded': False
                        }
                        
                        # 🚀 优化：不在这里调用get_bag_info()，改为按需加载
                        # 这大大提高了扫描速度
                        
                        bag_files.append(bag_info)
                        
                    except Exception as e:
                        self.get_logger().warning(f'Failed to read bag file information {item}: {str(e)}')
            
            # 按修改时间排序（最新的在前）
            bag_files.sort(key=lambda x: x.get('modified_time', 0), reverse=True)
            
        except Exception as e:
            self.get_logger().error(f'Failed to scan bag directory: {str(e)}')
        
        self.get_logger().info(f'Quick scan completed, found {len(bag_files)} bag files')
        return bag_files
    
    def get_bag_info(self, bag_path: str, fast_mode: bool = True) -> Dict[str, Any]:
        """获取bag文件详细信息
        
        Args:
            bag_path: bag文件路径
            fast_mode: 是否使用快速模式（只读取少量消息来估算信息）
        """
        try:
            from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
            import yaml
            
            if not os.path.exists(bag_path):
                return None
            
            # 🚀 缓存机制：检查是否已有缓存的信息
            cache_key = f"{bag_path}:{fast_mode}"
            bag_stat = Path(bag_path).stat()
            current_mtime = bag_stat.st_mtime
            
            if cache_key in self.bag_info_cache:
                cached_info, cached_mtime = self.bag_info_cache[cache_key]
                # 如果文件未修改，直接返回缓存
                if cached_mtime == current_mtime:
                    self.get_logger().debug(f'Using cached information: {bag_path}')
                    return cached_info
            
            # 优化1: 首先尝试从metadata.yaml读取基本信息
            metadata_path = Path(bag_path) / 'metadata.yaml'
            metadata_info = {}
            if metadata_path.exists():
                try:
                    with open(metadata_path, 'r') as f:
                        metadata = yaml.safe_load(f)
                        
                    # 从metadata获取基本信息
                    if 'rosbag2_bagfile_information' in metadata:
                        bag_info = metadata['rosbag2_bagfile_information']
                        topics_info = bag_info.get('topics_with_message_count', [])
                        
                        metadata_info = {
                            'topics': [topic['topic_metadata']['name'] for topic in topics_info],
                            'topic_count': len(topics_info),
                            'message_count': sum(topic['message_count'] for topic in topics_info),
                            'duration': bag_info.get('duration', {}).get('nanoseconds', 0) / 1e9,
                            'start_time': bag_info.get('starting_time', {}).get('nanoseconds_since_epoch', 0) / 1e9,
                        }
                        metadata_info['end_time'] = metadata_info['start_time'] + metadata_info['duration']
                        
                        # 🚀 Save metadata information to cache
                        self.bag_info_cache[cache_key] = (metadata_info, current_mtime)
                        
                        self.get_logger().debug(f'Successfully read information from metadata.yaml: {bag_path}')
                        return metadata_info
                        
                except Exception as e:
                    self.get_logger().debug(f'Failed to read metadata.yaml, using fallback method: {str(e)}')
            
            # 优化2: 如果metadata读取失败，使用快速采样模式
            reader = SequentialReader()
            storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            reader.open(storage_options, converter_options)
            
            # 统计信息
            topics = set()
            message_count = 0
            start_time = None
            end_time = None
            
            if fast_mode:
                # 快速模式：只读取前1000条消息和最后部分消息来估算
                sample_count = 0
                max_samples = 1000
                
                # 先读取一些消息获取开始时间和话题信息
                while reader.has_next() and sample_count < max_samples:
                    topic, data, timestamp = reader.read_next()
                    topics.add(topic)
                    message_count += 1
                    sample_count += 1
                    
                    if start_time is None or timestamp < start_time:
                        start_time = timestamp
                    if end_time is None or timestamp > end_time:
                        end_time = timestamp
                
                # 如果还有更多消息，快速跳过中间部分，只读取最后几条获取结束时间
                if reader.has_next():
                    # 估算总消息数（基于采样）
                    last_samples = []
                    skip_count = 0
                    
                    while reader.has_next():
                        topic, data, timestamp = reader.read_next()
                        message_count += 1
                        skip_count += 1
                        
                        # 只保留最后几条消息的时间戳
                        last_samples.append(timestamp)
                        if len(last_samples) > 10:
                            last_samples.pop(0)
                        
                        # Update log every 1000 messages
                        if skip_count % 1000 == 0:
                            self.get_logger().debug(f'Processing messages: {message_count}')
                    
                    # 使用最后的时间戳作为结束时间
                    if last_samples:
                        end_time = max(last_samples)
                
            else:
                # 完整模式：读取所有消息（原来的逻辑）
                while reader.has_next():
                    topic, data, timestamp = reader.read_next()
                    topics.add(topic)
                    message_count += 1
                    
                    if start_time is None or timestamp < start_time:
                        start_time = timestamp
                    if end_time is None or timestamp > end_time:
                        end_time = timestamp
            
            duration = 0.0
            if start_time and end_time:
                duration = (end_time - start_time) / 1e9  # 转换为秒
            
            result = {
                'topics': list(topics),
                'topic_count': len(topics),
                'message_count': message_count,
                'duration': duration,
                'start_time': start_time / 1e9 if start_time else 0,
                'end_time': end_time / 1e9 if end_time else 0,
                'fast_mode_used': fast_mode
            }
            
            # 🚀 Save to cache
            self.bag_info_cache[cache_key] = (result, current_mtime)
            
            self.get_logger().debug(f'Finished reading bag information: {bag_path}, message count: {message_count}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'Failed to read bag file information {bag_path}: {str(e)}')
            return None
    
    def get_playback_status(self) -> Dict[str, Any]:
        """获取当前回放状态"""
        return self.current_status
    
    def control_playback(self, command: Dict[str, Any]) -> bool:
        """发送回放控制命令"""
        try:
            control_msg = String()
            control_msg.data = json.dumps(command)
            self.control_publisher.publish(control_msg)
            
            self.get_logger().info(f'Sent playback control command: {command}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to send playback control command: {str(e)}')
            return False
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlaybackService()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 
