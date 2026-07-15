#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String, Float32, Bool, Int32, Float32MultiArray
from marvin_msgs.srv import Int as IntSrv
from sensor_msgs.msg import JointState
import threading
import time
import json
import os
from typing import List
import numpy as np
from marvin_msgs.msg import Jointcmd
from marvin_msgs.srv import MoveJ


class BagPlaybackNode(Node):
    """
    ROS2 Bag回放节点
    支持播放、暂停、停止、跳转、变速等功能

    回放策略：
    1. 机械臂只直接回放 bag 中记录的:
       - /control/joint_cmd_A
       - /control/joint_cmd_B
    2. /joint_states 不参与机械臂回放或初始化
    """

    def __init__(self):
        super().__init__('bag_playback_node')

        # Topic whitelist - only topics in the whitelist will be loaded and played
        self.declare_parameter(
            'topic_whitelist',
            [
                '/control/joint_cmd_A',
                '/control/joint_cmd_B',
                '/info/gripper_feedback_L',
                '/info/gripper_feedback_R',
                '/hand_left/joint_commands',
                '/hand_right/joint_commands'
            ]
        )
        self.topic_whitelist = self.get_parameter('topic_whitelist').get_parameter_value().string_array_value
        self.declare_parameter('storage_id', 'mcap')
        self.declare_parameter('playback_rate_default', 1.0)
        self.declare_parameter('playback_rate_min', 0.1)
        self.declare_parameter('playback_rate_max', 5.0)
        self.declare_parameter('initial_move_wait_sec', 5.0)
        self.declare_parameter('mode_request_retry_sec', 1.0)
        self.declare_parameter('mode_request_warn_sec', 5.0)

        self.storage_id = self.get_parameter('storage_id').value
        self.playback_rate_min = float(self.get_parameter('playback_rate_min').value)
        self.playback_rate_max = float(self.get_parameter('playback_rate_max').value)
        self.initial_move_wait_sec = float(self.get_parameter('initial_move_wait_sec').value)
        self.mode_request_retry_sec = float(self.get_parameter('mode_request_retry_sec').value)
        self.mode_request_warn_sec = float(self.get_parameter('mode_request_warn_sec').value)

        self.get_logger().info(f'Topic whitelist has been set: {self.topic_whitelist}')

        # State variables
        self.is_playing = False
        self.is_paused = False
        self.playback_rate = float(self.get_parameter('playback_rate_default').value)
        self.current_bag_path = None
        self.reader = None
        self.messages = []
        self.current_message_index = 0
        self.start_timestamp = None
        self.bag_duration = 0.0
        self.current_time = 0.0

        # Bag topic capability flags
        self.has_joint_cmd_A = False
        self.has_joint_cmd_B = False
        self.has_direct_joint_cmd = False

        # Thread control
        self.playback_thread = None
        self.stop_event = threading.Event()

        # Publisher dictionary
        self.topic_publishers = {}
        self.topic_types = {}

        # Create Jointcmd publishers
        self.jointcmd_publisher_L = self.create_publisher(
            Jointcmd,
            '/control/joint_cmd_A_playback',
            qos_profile_sensor_data
        )
        self.jointcmd_publisher_R = self.create_publisher(
            Jointcmd,
            '/control/joint_cmd_B_playback',
            qos_profile_sensor_data
        )

        self.hand_left_joint_commands_publisher = self.create_publisher(
            JointState,
            'hand_left/joint_commands_playback',
            qos_profile_sensor_data
        )
        self.hand_right_joint_commands_publisher = self.create_publisher(
            JointState,
            'hand_right/joint_commands_playback',
            qos_profile_sensor_data
        )

        # Create Gripper publishers
        self.grippercmd_publisher_L = self.create_publisher(
            Float32,
            'control/gripperValueL',
            10
        )
        self.grippercmd_publisher_R = self.create_publisher(
            Float32,
            'control/gripperValueR',
            10
        )

        self.get_logger().info('Jointcmd, gripper, and hand joint command publishers created')

        # Create MoveJ service client
        self.movej_client = self.create_client(MoveJ, 'control/movej')
        self.get_logger().info('MoveJ service client created')

        # Subscribe to control commands
        self.control_subscription = self.create_subscription(
            String,
            'control/playback_control',
            self.control_callback,
            10
        )

        self.request_mode_client = self.create_client(IntSrv, 'control/request_mode')
        self.sys_state_sub = self.create_subscription(
            Int32,
            'control/switch_state',
            self.sys_state_callback,
            10
        )

        # Publish playback status
        self.status_publisher = self.create_publisher(
            String,
            '/playback_status',
            10
        )

        playback_key_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.playback_key_publisher = self.create_publisher(
            Bool,
            '/playback_key',
            playback_key_qos
        )

        self.sys_state = 0
        self.last_mode_request_time = 0.0
        self.last_mode_warn_time = 0.0
        self.requested_mode = None

        # Status publishing timer
        self.status_timer = self.create_timer(0.1, self.publish_status)
        self.publish_playback_key(False)

        self.get_logger().info('Bag playback node started')

    def sys_state_callback(self, msg):
        """Handle system state update"""
        self.sys_state = msg.data
        self.get_logger().info(f'System state updated: {self.sys_state}')

    def request_mode(self, mode: int):
        now = time.time()
        if (
            self.requested_mode == mode and
            now - self.last_mode_request_time < self.mode_request_retry_sec
        ):
            return

        self.requested_mode = mode
        self.last_mode_request_time = now

        if not self.request_mode_client.service_is_ready():
            if now - self.last_mode_warn_time >= self.mode_request_warn_sec:
                self.get_logger().warn('control/request_mode not available')
                self.last_mode_warn_time = now
            return
        req = IntSrv.Request()
        req.data = mode
        self.request_mode_client.call_async(req)

    def control_callback(self, msg):
        """Handle control command from frontend"""
        try:
            command = json.loads(msg.data)
            cmd_type = command.get('type')

            self.get_logger().info(f'Received control command: {command}')

            if cmd_type == 'load':
                bag_path = command.get('bag_path')
                self.get_logger().info(f'Attempting to load bag file: {bag_path} (type: {type(bag_path)})')
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
            self.get_logger().error(f'Invalid control command format: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing control command: {str(e)}')

    def load_bag(self, bag_path):
        """Load bag file"""
        try:
            if not bag_path:
                self.get_logger().error('Bag file path cannot be empty')
                return False

            if not isinstance(bag_path, str):
                self.get_logger().error(f'Bag file path must be a string type, current type: {type(bag_path)}')
                return False

            if not os.path.exists(bag_path):
                self.get_logger().error(f'Bag file does not exist: {bag_path}')
                return False

            # Stop current playback
            self.stop()

            # Reset capability flags
            self.has_joint_cmd_A = False
            self.has_joint_cmd_B = False
            self.has_direct_joint_cmd = False

            # Initialize reader
            self.reader = SequentialReader()
            storage_options = StorageOptions(uri=bag_path, storage_id=self.storage_id)
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            self.reader.open(storage_options, converter_options)

            # Read all messages into memory, only load topics in whitelist
            self.messages = []
            self.topic_publishers.clear()

            total_messages = 0
            filtered_messages = 0

            while self.reader.has_next():
                topic, data, timestamp = self.reader.read_next()
                total_messages += 1

                if not self.topic_whitelist or topic in self.topic_whitelist:
                    self.messages.append((topic, data, timestamp))
                    filtered_messages += 1

                    if topic == '/control/joint_cmd_A':
                        self.has_joint_cmd_A = True
                    elif topic == '/control/joint_cmd_B':
                        self.has_joint_cmd_B = True

            self.has_direct_joint_cmd = self.has_joint_cmd_A and self.has_joint_cmd_B

            self.get_logger().info(
                f'Topic filter statistics: total messages={total_messages}, '
                f'loaded (whitelist)={filtered_messages}, filtered={total_messages - filtered_messages}'
            )
            self.get_logger().info(
                f'Bag topic capability: has_joint_cmd_A={self.has_joint_cmd_A}, '
                f'has_joint_cmd_B={self.has_joint_cmd_B}, '
                f'has_direct_joint_cmd={self.has_direct_joint_cmd}'
            )

            if self.messages:
                self.start_timestamp = self.messages[0][2]
                end_timestamp = self.messages[-1][2]
                self.bag_duration = (end_timestamp - self.start_timestamp) / 1e9

                self.current_bag_path = bag_path
                self.current_message_index = 0
                self.current_time = 0.0

                self.prepare_publishers()

                self.get_logger().info(f'Successfully loaded bag file: {bag_path}')
                self.get_logger().info(f'Message count: {len(self.messages)}, duration: {self.bag_duration:.2f}s')

                if self.has_direct_joint_cmd:
                    self.get_logger().info('Playback mode: direct playback of recorded /control/joint_cmd_A and /control/joint_cmd_B')
                else:
                    self.get_logger().warning('No complete /control/joint_cmd_A/B pair found in bag')

                return True
            else:
                self.get_logger().warning(f'Bag file is empty: {bag_path}')
                return False

        except Exception as e:
            self.get_logger().error(f'Failed to load bag file: {str(e)}')
            return False

    def prepare_publishers(self):
        """Prepare topic type info"""
        if not self.current_bag_path:
            self.get_logger().error('No current bag file path')
            return

        topic_types = {}

        reader = SequentialReader()
        storage_options = StorageOptions(uri=self.current_bag_path, storage_id=self.storage_id)
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)

        topic_metadata = reader.get_all_topics_and_types()
        self.get_logger().info(f'Found {len(topic_metadata)} topics')

        for topic_info in topic_metadata:
            topic_name = topic_info.name
            topic_type = topic_info.type

            try:
                msg_class = get_message(topic_type)
                topic_types[topic_name] = {
                    'type': topic_type,
                    'class': msg_class,
                }
                self.get_logger().info(f'Create publisher: {topic_name} ({topic_type})')
            except Exception as e:
                self.get_logger().error(f'Failed to create publisher {topic_name} ({topic_type}): {str(e)}')

        self.topic_types = topic_types

    def play(self):
        """Start playback"""
        if not self.messages:
            self.get_logger().warning('No bag file loaded')
            return

        if self.is_playing and not self.is_paused:
            return

        self.is_playing = True
        self.is_paused = False

        if self.playback_thread is None or not self.playback_thread.is_alive():
            self.stop_event.clear()
            self.requested_mode = None
            self.request_mode(3)
            self.playback_thread = threading.Thread(target=self.playback_loop)
            self.playback_thread.start()

        self.get_logger().info('Playback started')

    def pause(self):
        """Pause playback"""
        self.is_paused = not self.is_paused
        status = "Paused" if self.is_paused else "Resumed"
        self.get_logger().info(status)

    def stop(self):
        """Stop playback"""
        self.is_playing = False
        self.is_paused = False
        self.stop_event.set()

        if (
            self.playback_thread and
            self.playback_thread.is_alive() and
            threading.current_thread() != self.playback_thread
        ):
            self.playback_thread.join(timeout=1.0)

        self.current_message_index = 0
        self.current_time = 0.0
        self.get_logger().info('Playback stopped')

    def seek(self, target_time):
        """Seek to specified time (in seconds)"""
        if not self.messages:
            return

        if self.start_timestamp is not None:
            target_timestamp = self.start_timestamp + int(target_time * 1e9)
        else:
            return

        for i, (topic, data, timestamp) in enumerate(self.messages):
            if timestamp >= target_timestamp:
                self.current_message_index = i
                self.current_time = target_time
                break

        self.get_logger().info(f'Sought to time: {target_time:.2f}s')

    def set_playback_rate(self, rate):
        """Set playback speed"""
        self.playback_rate = max(self.playback_rate_min, min(rate, self.playback_rate_max))
        self.get_logger().info(f'Playback speed set: {self.playback_rate}x')

    def set_topic_whitelist(self, whitelist: List[str]):
        """Dynamically set topic whitelist"""
        if not isinstance(whitelist, list):
            self.get_logger().error(f'Whitelist must be a list type, current type: {type(whitelist)}')
            return

        self.topic_whitelist = whitelist
        self.get_logger().info(f'Topic whitelist updated: {self.topic_whitelist}')

        if self.current_bag_path:
            self.get_logger().warning('Whitelist updated, need to reload bag file to take effect')

    def publish_playback_key(self, is_playback_data: bool):
        """Publish whether current outgoing data is replayed bag data."""
        playback_key_msg = Bool()
        playback_key_msg.data = is_playback_data
        self.playback_key_publisher.publish(playback_key_msg)

    def move_to_init_positions(self, init_positions):
        """Move robot to initial state using a 14-element joint array."""
        if init_positions is None:
            self.get_logger().error('Initial joint positions are empty')
            return False

        init_positions = np.array(init_positions, dtype=float)
        if len(init_positions) != 14:
            self.get_logger().error(f'Initial joint position length must be 14, actual: {len(init_positions)}')
            return False

        self.get_logger().info(f'Initial positions: {init_positions}')

        try:
            movej_request = MoveJ.Request()
            movej_request.joint_values = init_positions.tolist()
            self.movej_client.wait_for_service(timeout_sec=5.0)
            self.movej_client.call_async(movej_request)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to publish initial positions: {str(e)}')
        return False

    def extract_init_positions_from_joint_cmds(self):
        """Extract initial arm state from first recorded /control/joint_cmd_A/B messages."""
        first_cmd_A = None
        first_cmd_B = None

        for topic, data, timestamp in self.messages:
            if topic not in ('/control/joint_cmd_A', '/control/joint_cmd_B'):
                continue

            try:
                msg_class = self.topic_types[topic]['class']
                msg = deserialize_message(data, msg_class)
            except Exception as e:
                self.get_logger().error(f'Failed to parse first Jointcmd message [{topic}]: {str(e)}')
                continue

            if not isinstance(msg, Jointcmd):
                self.get_logger().warning(f'Ignoring non-Jointcmd message on {topic}: {type(msg)}')
                continue

            if topic == '/control/joint_cmd_A' and first_cmd_A is None:
                first_cmd_A = msg
            elif topic == '/control/joint_cmd_B' and first_cmd_B is None:
                first_cmd_B = msg

            if first_cmd_A is not None and first_cmd_B is not None:
                break

        if first_cmd_A is None or first_cmd_B is None:
            return None

        if len(first_cmd_A.positions) != 7 or len(first_cmd_B.positions) != 7:
            self.get_logger().warning(
                f'Initial Jointcmd length invalid: A={len(first_cmd_A.positions)}, '
                f'B={len(first_cmd_B.positions)}'
            )
            return None

        return np.array(list(first_cmd_A.positions) + list(first_cmd_B.positions))

    def playback_loop(self):
        """Playback loop (runs in separate thread)"""
        if self.current_message_index == 0:
            self.get_logger().info('Preparing to move robot to initial state...')

            init_positions = None
            if self.has_direct_joint_cmd:
                init_positions = self.extract_init_positions_from_joint_cmds()
                if init_positions is not None:
                    self.get_logger().info('Using first recorded /control/joint_cmd_A/B as initial state')

            if init_positions is not None and self.move_to_init_positions(init_positions):
                self.get_logger().info(f'Waiting {self.initial_move_wait_sec} seconds before publishing jointcmd...')
                time.sleep(self.initial_move_wait_sec)
            else:
                self.get_logger().warning('Failed to move to initial state, continuing playback')

        playback_start_time = time.time()
        bag_start_time = self.start_timestamp

        if self.current_message_index > 0:
            current_msg_timestamp = self.messages[self.current_message_index][2]
            bag_start_time = current_msg_timestamp
            self.current_time = (current_msg_timestamp - self.start_timestamp) / 1e9

        self.get_logger().info(
            f'Playback started from index {self.current_message_index}, current time: {self.current_time:.3f}s'
        )

        while (
            self.is_playing and
            self.current_message_index < len(self.messages) and
            not self.stop_event.is_set()
        ):
            if self.sys_state != 0 and self.sys_state != 3:
                time.sleep(0.01)
                playback_start_time = time.time()
                if self.current_message_index < len(self.messages):
                    bag_start_time = self.messages[self.current_message_index][2]
                continue

            if self.sys_state != 3:
                self.request_mode(3)

            if self.is_paused:
                time.sleep(0.01)
                playback_start_time = time.time()
                if self.current_message_index < len(self.messages):
                    bag_start_time = self.messages[self.current_message_index][2]
                continue

            topic, data, timestamp = self.messages[self.current_message_index]

            msg_relative_time = (timestamp - bag_start_time) / 1e9
            target_playback_time = playback_start_time + (msg_relative_time / self.playback_rate)
            current_real_time = time.time()

            if current_real_time < target_playback_time:
                sleep_time = target_playback_time - current_real_time
                if sleep_time > 0.001:
                    time.sleep(min(sleep_time, 0.1))
                continue

            self.publish_message(topic, data)
            self.current_time = (timestamp - self.start_timestamp) / 1e9
            self.current_message_index += 1

            if self.current_message_index % 10 == 0:
                self.get_logger().debug(
                    f'Playback progress: {self.current_message_index}/{len(self.messages)}, '
                    f'time: {self.current_time:.3f}s'
                )

        if self.current_message_index >= len(self.messages):
            self.get_logger().info('Playback completed')

            self.is_playing = False
            self.is_paused = False
            self.current_message_index = 0
            self.current_time = 0.0
            self.publish_status()
            return

    def publish_message(self, topic, data):
        """Publish message to corresponding topic"""
        try:
            if topic not in self.topic_types:
                self.get_logger().warning(f'Topic type info not found for topic: {topic}')
                return

            msg_class = self.topic_types[topic]['class']
            msg = deserialize_message(data, msg_class)

            # 1) Direct playback of recorded joint_cmd_A/B.
            if topic == '/control/joint_cmd_A' and isinstance(msg, Jointcmd):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.jointcmd_publisher_L.publish(msg)
                return

            if topic == '/control/joint_cmd_B' and isinstance(msg, Jointcmd):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.jointcmd_publisher_R.publish(msg)
                return

            # 2) hand joint commands playback
            if topic == '/hand_left/joint_commands' and isinstance(msg, JointState):
                self.hand_left_joint_commands_publisher.publish(msg)
                return

            if topic == '/hand_right/joint_commands' and isinstance(msg, JointState):
                self.hand_right_joint_commands_publisher.publish(msg)
                return

            # 3) gripper feedback remap playback
            if topic == '/info/gripper_feedback_L' and isinstance(msg, Float32MultiArray):
                if len(msg.data) > 0:
                    gripper_msg_L = Float32()
                    gripper_msg_L.data = msg.data[0]
                    self.grippercmd_publisher_L.publish(gripper_msg_L)
                else:
                    self.get_logger().warning('/info/gripper_feedback_L data is empty')
                return

            if topic == '/info/gripper_feedback_R' and isinstance(msg, Float32MultiArray):
                if len(msg.data) > 0:
                    gripper_msg_R = Float32()
                    gripper_msg_R.data = msg.data[0]
                    self.grippercmd_publisher_R.publish(gripper_msg_R)
                else:
                    self.get_logger().warning('/info/gripper_feedback_R data is empty')
                return

        except Exception as e:
            if hasattr(self, 'topic_types') and topic in self.topic_types:
                topic_type = self.topic_types[topic]['type']
                self.get_logger().error(f'Failed to publish message [{topic}] ({topic_type}): {str(e)}')
            else:
                self.get_logger().error(f'Failed to publish message [{topic}]: {str(e)}')

    def publish_status(self):
        """发布回放状态"""
        is_playback_data = (
            self.is_playing and
            not self.is_paused and
            self.current_message_index < len(self.messages) and
            self.sys_state in (0, 3)
        )

        status = {
            'is_playing': self.is_playing,
            'is_paused': self.is_paused,
            'playback_rate': self.playback_rate,
            'current_time': self.current_time,
            'duration': self.bag_duration,
            'progress': self.current_time / self.bag_duration if self.bag_duration > 0 else 0.0,
            'bag_path': self.current_bag_path,
            'has_joint_cmd_A': self.has_joint_cmd_A,
            'has_joint_cmd_B': self.has_joint_cmd_B,
            'has_direct_joint_cmd': self.has_direct_joint_cmd
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)
        self.publish_playback_key(is_playback_data)


def main(args=None):
    rclpy.init(args=args)
    node = BagPlaybackNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
