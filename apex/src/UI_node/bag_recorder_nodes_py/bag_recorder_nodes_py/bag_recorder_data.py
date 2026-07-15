import sys
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.parameter import Parameter
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger,SetBool
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from rclpy.qos import qos_profile_sensor_data
import importlib
from pydantic import BaseModel
from typing import List

import rosbag2_py
import threading
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import time
from datetime import datetime
import glob
import os
import termios
from typing import Optional
from marvin_msgs.srv import VideoCapture
# 统一FastAPI：在同一服务内管理视频与rosbag
# 尝试导入 Jetson 视频工具（若不可用则禁用视频服务）


app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 或指定你的前端地址
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
recorder_node = None
storage_error_message = None  # 存储路径错误消息
# 统一服务中的视频对象与状态

ALLOWED_TOPICS = {
    "/joint_states",
    "/info/eef_left",
    "/info/eef_right",
    "/control/target_poseL",
    "/control/target_poseR",
    "/control/joint_cmd_A",
    "/control/joint_cmd_B",
    "/hand_left/joint_commands",
    "/hand_left/joint_states",
    "/hand_right/joint_commands",
    "/hand_right/joint_states",

    # for realsense
    "/camera/camera/aligned_depth_to_color/camera_info",
    "/camera/camera/color/image_raw",
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera/color/camera_info",

    # left wrist RealSense D405
    "/wrist_left/wrist_left/aligned_depth_to_color/camera_info",
    "/wrist_left/wrist_left/color/image_raw",
    "/wrist_left/wrist_left/color/image_rect_raw",
    "/wrist_left/wrist_left/aligned_depth_to_color/image_raw",
    "/wrist_left/wrist_left/color/camera_info",

    # right wrist RealSense D405
    "/wrist_right/wrist_right/aligned_depth_to_color/camera_info",
    "/wrist_right/wrist_right/color/image_raw",
    "/wrist_right/wrist_right/color/image_rect_raw",
    "/wrist_right/wrist_right/aligned_depth_to_color/image_raw",
    "/wrist_right/wrist_right/color/camera_info",
}

CAMERA_TOPIC_GROUPS = (
    ("main color image", ("/camera/camera/color/image_raw",)),
    ("main aligned depth image", ("/camera/camera/aligned_depth_to_color/image_raw",)),
    ("main color camera_info", ("/camera/camera/color/camera_info",)),
    ("main depth camera_info", ("/camera/camera/aligned_depth_to_color/camera_info",)),
    (
        "left wrist color image",
        (
            "/wrist_left/wrist_left/color/image_raw",
            "/wrist_left/wrist_left/color/image_rect_raw",
        ),
    ),
    (
        "left wrist aligned depth image",
        ("/wrist_left/wrist_left/aligned_depth_to_color/image_raw",),
    ),
    (
        "left wrist color camera_info",
        ("/wrist_left/wrist_left/color/camera_info",),
    ),
    (
        "left wrist depth camera_info",
        ("/wrist_left/wrist_left/aligned_depth_to_color/camera_info",),
    ),
    (
        "right wrist color image",
        (
            "/wrist_right/wrist_right/color/image_raw",
            "/wrist_right/wrist_right/color/image_rect_raw",
        ),
    ),
    (
        "right wrist aligned depth image",
        ("/wrist_right/wrist_right/aligned_depth_to_color/image_raw",),
    ),
    (
        "right wrist color camera_info",
        ("/wrist_right/wrist_right/color/camera_info",),
    ),
    (
        "right wrist depth camera_info",
        ("/wrist_right/wrist_right/aligned_depth_to_color/camera_info",),
    ),
)


class UsbWarningLight:
    """Control Hongming USB serial warning lights without external dependencies."""

    COLOR_ADDR = {
        "green": 0x02,
        "red": 0x03,
        "yellow": 0x04,
        "blue": 0x09,
    }

    OP = {
        "off": 0x00,
        "on": 0x01,
        "blink": 0x02,
    }

    def __init__(self, node: Node):
        self.node = node
        self.enabled = bool(node.get_parameter("warning_light_enabled").value)
        self.port = str(node.get_parameter("warning_light_port").value)
        self.baud = int(node.get_parameter("warning_light_baud").value)
        self.recording_color = str(node.get_parameter("warning_light_recording_color").value).lower()
        self.stopped_color = str(node.get_parameter("warning_light_stopped_color").value).lower()
        self._warned = False
        if self.port.lower() == "auto":
            self.port = self._find_port()

    def set_recording(self):
        self.set_color(self.recording_color)

    def set_stopped(self):
        self.set_color(self.stopped_color)

    def set_color(self, color: str):
        if not self.enabled:
            return
        if color in ("", "none", "off"):
            self.all_off()
            return
        if color not in self.COLOR_ADDR:
            self._warn_once(f"Unsupported warning light color: {color}")
            return
        self.all_off()
        time.sleep(0.12)
        self._send(self.COLOR_ADDR[color], self.OP["on"])

    def all_off(self):
        if self.enabled:
            self._send(0x00, self.OP["off"])

    def _find_port(self):
        if os.path.exists("/dev/apex_warning_light"):
            return "/dev/apex_warning_light"
        for path in glob.glob("/dev/serial/by-id/*1a86*") + glob.glob("/dev/serial/by-id/*CH34*"):
            return path
        ports = sorted(glob.glob("/dev/ttyUSB*"))
        if ports:
            return ports[0]
        return "/dev/ttyUSB0"

    def _send(self, address: int, op: int):
        checksum = (0xA0 + address + op) & 0xFF
        payload = bytes((0xA0, address, op, checksum))
        fd = None
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self._configure(fd)
            os.write(fd, payload)
        except OSError as exc:
            self._warn_once(f"Warning light write failed on {self.port}: {exc}")
        finally:
            if fd is not None:
                os.close(fd)

    def _configure(self, fd):
        attrs = termios.tcgetattr(fd)
        baud_attr = getattr(termios, f"B{self.baud}", termios.B9600)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud_attr
        attrs[5] = baud_attr
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def _warn_once(self, message: str):
        if not self._warned:
            self.node.get_logger().warn(message)
            self._warned = True

# 定义请求体模型
class TopicSelectionModel(BaseModel):
    topics: List[str]

# 获取话题列表API
@app.get("/get_topics")
def get_topics():
    if recorder_node is not None:
        topics = recorder_node.get_available_topics()
        return {"topics": topics}
    return {"topics": [], "error": "No recorder node available"}

# 修改录制API接收话题列表
@app.post("/start_record")
def start_record(topic_selection: TopicSelectionModel = None):
    if recorder_node is not None and not recorder_node.recording:
        try:
            if topic_selection and topic_selection.topics:
                recorder_node.start_recording(topic_selection.topics)
            else:
                recorder_node.start_recording()  # 如果没有指定话题，录制所有话题
            recorder_node._triggered_by_service = False  # 标记为FastAPI触发，非服务触发
            # 同步启动视频录制（如无需联动，可注释）
            # place holder for vid
            
            return {"status": "started"}
        except FileNotFoundError as e:
            return {"status": "error", "error_message": str(e)}
        except Exception as e:
            return {"status": "error", "error_message": f"启动录制失败: {str(e)}"}
    return {"status": "already recording or no node"}

@app.post("/stop_record")
def stop_record():
    if recorder_node is not None and recorder_node.recording:
        if recorder_node._video_recording_enabled():
            recorder_node.video_record_srv.call_async(VideoCapture.Request(start_stop=False))
        recorder_node.stop_recording()  # 停止录制并关闭当前rosbag文件
        # 同步停止视频录制（如无需联动，可注释）
        return {"status": "stopped"}
    return {"status": "not recording or no node"}

# 获取录制状态API
@app.get("/record_status")
def get_record_status():
    if recorder_node is not None:
        return {
            "is_recording": recorder_node.recording,
            "topics": recorder_node._selected_topics if recorder_node._selected_topics else [],
            "triggered_by_service": getattr(recorder_node, '_triggered_by_service', False)
        }
    return {
        "is_recording": False,
        "topics": [],
        "triggered_by_service": False
    }

# 获取存储状态API
@app.get("/storage_status")
def get_storage_status():
    global storage_error_message
    if storage_error_message:
        return {
            "has_error": True,
            "error_message": storage_error_message
        }
    return {
        "has_error": False,
        "error_message": None
    }

# 清除存储错误状态API
@app.post("/clear_storage_error")
def clear_storage_error():
    global storage_error_message
    storage_error_message = None
    return {"status": "cleared"}



class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.recording = False
        self.writer = None
        self._topic_subscriptions = []  # 修改属性名称，避免与Node类内部属性冲突
        self._selected_topics = []  # 存储用户选择的话题
        self._triggered_by_service = False  # 标记是否由ROS服务触发录制
        
        # 声明参数，设置默认的存储路径（指向当前项目下BAG_STORAGE/recorded_bags）
        default_storage = '/media/marvin/BAG_STORAGE/recorded_bags'
        self.declare_parameter('record_bags_storage_dir', default_storage)
        self.get_logger().info(f'Recording bags storage path parameter declared: {self.get_parameter("record_bags_storage_dir").value}')
        self.declare_parameter('enable_video_recording', False)
        self.declare_parameter('record_topic_discovery_wait_sec', 1.0)
        self.declare_parameter('warning_light_enabled', True)
        self.declare_parameter('warning_light_port', '/dev/ttyUSB0')
        self.declare_parameter('warning_light_baud', 9600)
        self.declare_parameter('warning_light_recording_color', 'red')
        self.declare_parameter('warning_light_stopped_color', 'green')
        self.get_logger().info(f'Video recording link enabled: {self._video_recording_enabled()}')
        self.warning_light = UsbWarningLight(self)
        self.warning_light.set_stopped()
        self.video_record_srv = self.create_client(VideoCapture, 'recorder/set_recording')
        self.recording_service = self.create_service(
            Trigger,
            'toggle_recording',
            self.toggle_recording_callback
        )
        self.set_recording_service = self.create_service(
            SetBool,
            '/data_bag_recorder/set_recording',
            self.set_recording_callback
        )
        self.get_logger().info(
            'Recording control services created: /toggle_recording, /data_bag_recorder/set_recording'
        )
        
        # 创建一个发布者，发布录制状态
        self.status_publisher = self.create_publisher(Int32, '/recorder/status', 10)
        # 创建一个定时器，以1Hz频率发布状态
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Status publisher created: /recorder/status (1Hz)')
    
    def toggle_recording_callback(self, request, response):
        """Toggle recording from ROS clients such as foot controls."""
        del request
        try:
            success, message = self._set_recording_state(
                should_record=not self.recording,
                triggered_by_service=True
            )
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"操作失败: {str(e)}"
            self.get_logger().error(f"Recording toggle service failed: {str(e)}")
        return response

    def set_recording_callback(self, request, response):
        """Set recording state explicitly from ROS clients."""
        try:
            success, message = self._set_recording_state(
                should_record=bool(request.data),
                triggered_by_service=True
            )
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"操作失败: {str(e)}"
            self.get_logger().error(f"Recording set service failed: {str(e)}")
        return response

    def _set_recording_state(self, should_record: bool, triggered_by_service: bool = False):
        if should_record:
            if self.recording:
                return True, "录制已在进行中"
            self.start_recording()
            self._triggered_by_service = triggered_by_service
            return True, "录制已开始"

        if not self.recording:
            return True, "录制已停止"
        if self._video_recording_enabled():
            self.video_record_srv.call_async(VideoCapture.Request(start_stop=False))
        self.stop_recording()
        return True, "录制已停止"
    
    def get_available_topics(self):
        """获取所有可用话题及其类型"""
        topic_list = []
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, type_list in topic_names_and_types:
            # 过滤掉隐藏话题
            if topic_or_service_is_hidden(topic_name):
                continue
            print(topic_name)
            # # 过滤掉包含/info/和/control/的话题
            # if '/info/' in topic_name or '/control/' in topic_name:
            #     self.get_logger().debug(f'过滤掉话题: {topic_name}')
            #     continue

            if not self._is_topic_allowed(topic_name):
                self.get_logger().debug(f'Filtering out non-whitelisted topic: {topic_name}')
                continue
                
            # 添加符合条件的话题
            topic_list.append({
                "name": topic_name,
                "type": type_list[0]
            })
        return topic_list

    def _is_topic_allowed(self, topic_name: str) -> bool:
        """仅允许白名单中的话题被录制"""
        return topic_name in ALLOWED_TOPICS

    def _video_recording_enabled(self) -> bool:
        """Whether to call the external camera recorder service."""
        return bool(self.get_parameter('enable_video_recording').value)

    def _wait_for_record_topics(self):
        """Give camera topics time to appear before freezing the bag topic set."""
        wait_sec = float(self.get_parameter('record_topic_discovery_wait_sec').value)
        if wait_sec <= 0.0:
            return

        deadline = time.monotonic() + wait_sec
        last_missing = None

        while time.monotonic() < deadline:
            current_topics = {
                topic_name
                for topic_name, _ in self.get_topic_names_and_types()
                if topic_name in ALLOWED_TOPICS and not topic_or_service_is_hidden(topic_name)
            }
            missing = [
                name
                for name, alternatives in CAMERA_TOPIC_GROUPS
                if not any(topic in current_topics for topic in alternatives)
            ]
            if not missing:
                return
            if missing != last_missing:
                self.get_logger().info(
                    f'Waiting for camera topics before recording; missing groups: {missing}'
                )
                last_missing = missing
            time.sleep(0.2)
    
    def _import_msg_type(self, msg_type_str):
        """
        动态导入消息类型
        例如: 'std_msgs/msg/String' -> std_msgs.msg.String类
        """
        try:
            parts = msg_type_str.split('/')
            module_name = '.'.join(parts[:-1])
            class_name = parts[-1]
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f'Failed to import message type {msg_type_str}: {str(e)}')
            return None

    def _create_topic_callback(self, topic_name):
        """
        为每个话题创建专属回调函数
        """
        def callback(msg):
            if self.recording and self.writer is not None:
                try:
                    self.writer.write(
                        topic_name,
                        serialize_message(msg),
                        self.get_clock().now().nanoseconds)
                except Exception as e:
                    self.get_logger().error(f'Failed to write topic {topic_name}: {str(e)}')
        return callback

    def start_recording(self, selected_topics=None):
        """开始录制指定话题，如果未指定则录制所有话题"""
        # 清理旧的订阅者
        self.destroy_subscriptions()
        
        # 保存选中的话题
        if selected_topics:
            allowed_selection = [t for t in selected_topics if self._is_topic_allowed(t)]
            dropped_topics = set(selected_topics) - set(allowed_selection)
            if dropped_topics:
                self.get_logger().info(f'Ignoring non-whitelisted topics: {dropped_topics}')
            self._selected_topics = allowed_selection
        else:
            self._selected_topics = None
        
        # 创建新的bag文件
        now = datetime.now()
        bag_name = now.strftime("my_bag-%y-%m-%d-%H-%M-%S")
        # 从参数服务器获取存储路径
        storage_dir = self.get_parameter('record_bags_storage_dir').value
        storage_dir = os.path.join(storage_dir,bag_name)
        bag_dir = os.path.join(storage_dir, 'data')
        video_dir = os.path.join(storage_dir, 'video')

        
        # 检查存储路径的父目录是否存在，不存在则尝试创建
        parent_dir = os.path.dirname(storage_dir)
        if not os.path.exists(parent_dir):
            try:
                os.makedirs(parent_dir, exist_ok=True)
                self.get_logger().info(f'Storage parent directory created: {parent_dir}')
            except Exception:
                global storage_error_message
                error_msg = 'Storage disk not found, please insert and retry'
                storage_error_message = error_msg
                self.get_logger().error(f'Storage parent directory does not exist: {parent_dir}')
                raise FileNotFoundError(error_msg)
        
        # 确保bag目录存在，便于视频录制落盘到同目录
        try:
            # os.makedirs(bag_dir, exist_ok=True)
            os.makedirs(video_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().warn(f'Failed to create storage directory: {str(e)}')

        # 同步设置视频录制目录为bag目录
        # try:
        #     if self.video_param_client.wait_for_service(timeout_sec=1.0):
        #         self.video_param_client.set_parameters([
        #             Parameter('mp4_dir', Parameter.Type.STRING, bag_dir)
        #         ])
        #     else:
        #         self.get_logger().warn('视频节点参数服务不可用，未同步mp4_dir')
        # except Exception as e:
        #     self.get_logger().warn(f'同步mp4_dir失败: {str(e)}')
        video_name = os.path.join(video_dir, 'cameras.mp4')
        vidcap = VideoCapture.Request()
        vidcap.start_stop = True
        vidcap.save_dir = video_name
        if self._video_recording_enabled():
            recorder_node.video_record_srv.call_async(vidcap)
        else:
            self.get_logger().info('Video recording disabled; recording ROS topics only')

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir,
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        
        self._wait_for_record_topics()

        # 获取所有当前存在的话题
        topic_count = 0
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, type_list in topic_names_and_types:
            # 跳过隐藏话题
            if topic_or_service_is_hidden(topic_name):
                continue

            # 仅录制白名单中的话题
            if not self._is_topic_allowed(topic_name):
                continue
            
            # 如果指定了话题列表，只处理列表中的话题
            if self._selected_topics and topic_name not in self._selected_topics:
                continue
                
            # 获取话题类型
            topic_type = type_list[0]
            
            # 创建话题元数据
            topic_info = rosbag2_py.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            
            # 在bag中注册话题
            try:
                self.writer.create_topic(topic_info)
                topic_count += 1
                
                # 动态导入消息类型
                msg_class = self._import_msg_type(topic_type)
                if msg_class is not None:
                    # 创建订阅者和对应回调
                    sub = self.create_subscription(
                        msg_class,
                        topic_name,
                        self._create_topic_callback(topic_name),
                        qos_profile_sensor_data)
                    self._topic_subscriptions.append(sub)  # Using new attribute name
                    self.get_logger().info(f'Topic subscribed: {topic_name} ({topic_type})')
            except Exception as e:
                self.get_logger().error(f'Failed to register topic {topic_name}: {str(e)}')
        
        self.recording = True
        self.warning_light.set_recording()
        if topic_count == 0:
            self.get_logger().warn(f"No whitelisted topics found, bag is empty: {bag_name}")
        else:
            self.get_logger().info(f"Starting recording: {bag_name} (registered {topic_count} topics)")

    def stop_recording(self):
        self.recording = False
        self._triggered_by_service = False  # Reset service trigger flag
        if self.writer is not None:
            # Clean up writer resources
            del self.writer
            self.writer = None
        # Optional: destroy subscriptions when stopping recording
        self.destroy_subscriptions()
        self.warning_light.set_stopped()
        self.get_logger().info("Recording stopped.")

    def shutdown(self):
        if self.recording:
            self.stop_recording()
        else:
            self.destroy_subscriptions()
        self.warning_light.all_off()

    def destroy_subscriptions(self):
        """Destroy all created subscriptions"""
        for sub in self._topic_subscriptions:
            self.destroy_subscription(sub)
        self._topic_subscriptions.clear()
        self.get_logger().info("All subscriptions destroyed")
        
    def publish_status(self):
        """Publish recording status, 1 for recording, 0 for stopped"""
        msg = Int32()
        msg.data = 1 if self.recording else 0
        self.status_publisher.publish(msg)
        # Only log when state changes to avoid excessive output
        # self.get_logger().debug(f"Publishing recording status: {'recording' if self.recording else 'stopped'}")




def ros2_thread():
    global recorder_node
    rclpy.init()
    recorder_node = SimpleBagRecorder()
    try:
        rclpy.spin(recorder_node)
    finally:
        if recorder_node is not None:
            recorder_node.shutdown()
            recorder_node.destroy_node()
            recorder_node = None
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    # 启动ROS2节点线程
    t = threading.Thread(target=ros2_thread, daemon=True)
    t.start()
    # 启动视频采集/推流线程（与同一FastAPI共享API）
    # 启动FastAPI服务
    uvicorn.run(app, host="0.0.0.0", port=8000, access_log=False)


if __name__ == '__main__':
    main()
