import importlib
import os
import shutil
from datetime import datetime
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from rclpy.serialization import serialize_message
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden

import rosbag2_py

DEFAULT_LOG_DIR = '/home/marvin/.ros/log'
DEFAULT_MAX_LOGS = 10
LOG_BAG_PREFIX = 'topic_log-'

# 白名单：仅记录这些话题
ALLOWED_TOPICS = {
    "/joint_states",
    "/info/gripper_feedback_L",
    "/info/gripper_feedback_R",
    "/info/eef_left",
    "/info/eef_right",
    "/control/Elbow_left",
    "/control/Elbow_right",
    "/control/eef_cmd_A",
    "/control/eef_cmd_B",
    "/control/enableL",
    "/control/enableR",
    "/control/gripperValueL",
    "/control/gripperValueR",
    "/control/ik_cmd_A",
    "/control/ik_cmd_B",
    "/control/ik_request",
    "/control/ik_result",
    "/control/joint_cmd_A",
    "/control/joint_cmd_B",
    "/control/joint_cmd_plan_A",
    "/control/joint_cmd_plan_B",
    "/control/joint_cmd_tele_A",
    "/control/joint_cmd_tele_B",
    "/control/playback_control",
    "/control/switch_state",
    "/control/target_poseL",
    "/control/target_poseR",
    "/hand_left/joint_commands",
    "/hand_left/joint_states",
    "/hand_right/joint_commands",
    "/hand_right/joint_states",
}


class AllTopicLogRecorder(Node):
    """Continuously record all visible topics into a rolling log bag."""

    def __init__(self):
        super().__init__('all_topic_log_recorder')
        self.declare_parameter('log_storage_dir', DEFAULT_LOG_DIR)
        self.declare_parameter('max_log_bags', DEFAULT_MAX_LOGS)
        self.declare_parameter('scan_interval_sec', 2.0)

        self._writer: Optional[rosbag2_py.SequentialWriter] = None
        self._recorder_subs: Dict[str, object] = {}  # 避免与 rclpy Node._subscriptions 冲突
        self._registered_topics = set()
        self._started = False

        self._startup_timer = self.create_timer(1.0, self._start_recording_once)
        self._scan_timer = self.create_timer(
            float(self.get_parameter('scan_interval_sec').value),
            self._refresh_topics,
        )

        self.get_logger().info('All topic log recorder initialized')

    def _start_recording_once(self):
        if self._started:
            return

        self._startup_timer.cancel()
        try:
            self._prepare_writer()
            self._refresh_topics()
            self._started = True
            self.get_logger().info('All topic log recording started')
        except Exception as error:
            self.get_logger().error(f'Failed to start log recorder: {error}')

    def _prepare_writer(self):
        log_storage_dir = self.get_parameter('log_storage_dir').value
        max_log_bags = int(self.get_parameter('max_log_bags').value)

        os.makedirs(log_storage_dir, exist_ok=True)
        self._rotate_old_logs(log_storage_dir, max_log_bags)

        bag_name = datetime.now().strftime(f'{LOG_BAG_PREFIX}%Y-%m-%d-%H-%M-%S')
        bag_path = os.path.join(log_storage_dir, bag_name)

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id='sqlite3',
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        writer.open(storage_options, converter_options)
        self._writer = writer

        self.get_logger().info(f'Writing log bag to: {bag_path}')

    def _rotate_old_logs(self, log_storage_dir: str, max_log_bags: int):
        entries = sorted(
            [
                entry for entry in os.scandir(log_storage_dir)
                if entry.is_dir() and entry.name.startswith(LOG_BAG_PREFIX)
            ],
            key=lambda entry: entry.stat().st_mtime,
        )

        while len(entries) >= max_log_bags:
            oldest_entry = entries.pop(0)
            shutil.rmtree(oldest_entry.path)
            self.get_logger().info(
                f'Removed oldest log bag: {oldest_entry.name}'
            )

    def _refresh_topics(self):
        if self._writer is None:
            return

        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, type_list in topic_names_and_types:
            if topic_or_service_is_hidden(topic_name):
                continue
            # 白名单过滤：仅记录允许的话题
            if topic_name not in ALLOWED_TOPICS:
                continue
            if not type_list or topic_name in self._registered_topics:
                continue

            topic_type = type_list[0]
            msg_type = self._import_msg_type(topic_type)
            if msg_type is None:
                continue

            # 验证消息类型是否有效
            if not hasattr(msg_type, '__slots__'):
                self.get_logger().warn(
                    f'Skipping topic {topic_name}: Invalid message type {topic_type}'
                )
                continue

            try:
                topic_info = rosbag2_py.TopicMetadata(
                    name=topic_name,
                    type=topic_type,
                    serialization_format='cdr',
                )
                self._writer.create_topic(topic_info)
                
                # 创建一个兼容性强的 QoS profile
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10,
                    liveliness=QoSLivelinessPolicy.AUTOMATIC
                )
                
                subscription = self.create_subscription(
                    msg_type,
                    topic_name,
                    self._build_callback(topic_name),
                    qos
                )
            except Exception as error:
                self.get_logger().warn(
                    f'Skipping topic {topic_name} ({topic_type}): {error}'
                )
                continue

            self._recorder_subs[topic_name] = subscription
            self._registered_topics.add(topic_name)
            self.get_logger().info(
                f'Logging topic: {topic_name} ({topic_type})'
            )

    def _import_msg_type(self, msg_type_str: str):
        try:
            module_path, _, class_name = msg_type_str.rpartition('/')
            module = importlib.import_module(module_path.replace('/', '.'))
            return getattr(module, class_name)
        except (ImportError, AttributeError, ValueError) as error:
            self.get_logger().warn(
                f'Failed to import message type {msg_type_str}: {error}'
            )
            return None

    def _get_qos_profile(self, topic_name: str):
        # 使用通用的兼容QoS配置，避免从publisher获取导致的转换错误
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _build_callback(self, topic_name: str):
        def callback(message):
            if self._writer is None:
                return

            try:
                self._writer.write(
                    topic_name,
                    serialize_message(message),
                    self.get_clock().now().nanoseconds,
                )
            except Exception as error:
                self.get_logger().error(
                    f'Failed to write topic {topic_name}: {error}'
                )

        return callback

    def shutdown_recorder(self):
        for subscription in self._recorder_subs.values():
            self.destroy_subscription(subscription)
        self._recorder_subs.clear()
        self._registered_topics.clear()

        if self._writer is not None:
            del self._writer
            self._writer = None


def main(args=None):
    rclpy.init(args=args)
    node = AllTopicLogRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_recorder()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
