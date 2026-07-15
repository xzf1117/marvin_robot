#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool, Int32
from std_srvs.srv import Trigger
import socket
import json
import threading
from rclpy.qos import qos_profile_sensor_data
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class PoseGripMsg:
    def __init__(self, px=0.0, py=0.0, pz=0.0,
                 qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        self.px = px
        self.py = py
        self.pz = pz
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw

    def pack(self):
        data = {
            "px": self.px,
            "py": self.py,
            "pz": self.pz,
            "qx": self.qx,
            "qy": self.qy,
            "qz": self.qz,
            "qw": self.qw,
        }
        return json.dumps(data).encode('utf-8')


class UDPPoseReceiver(Node):

    def __init__(self):
        super().__init__('udp_pose_receiver')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('glove_mode', False)
        self.declare_parameter('robot_ip', '192.168.1.100')
        self.declare_parameter('base_height', 1.0)
        self.declare_parameter('tcp_port', 9010)
        self.declare_parameter('tcp_heartbeat_timeout', 5.0)

        self.glove_mode           = self.get_parameter('glove_mode').value
        robot_ip                  = self.get_parameter('robot_ip').value
        self.base_height          = self.get_parameter('base_height').value
        tcp_port                  = self.get_parameter('tcp_port').value
        tcp_heartbeat_timeout     = self.get_parameter('tcp_heartbeat_timeout').value

        # ── Publishers ────────────────────────────────────────────────────────
        self.publisher_A     = self.create_publisher(PoseStamped, 'control/target_poseL',  qos_profile_sensor_data)
        self.trigger_pose_A  = self.create_publisher(Bool,        'control/enableL',        qos_profile_sensor_data)
        self.publisher_B     = self.create_publisher(PoseStamped, 'control/target_poseR',  qos_profile_sensor_data)
        self.trigger_pose_B  = self.create_publisher(Bool,        'control/enableR',        qos_profile_sensor_data)
        self.trigger_value_A = self.create_publisher(Float32,     'control/gripperValueL',  10)
        self.trigger_value_B = self.create_publisher(Float32,     'control/gripperValueR',  10)
        self.publisher_EL    = self.create_publisher(PoseStamped, 'control/Elbow_left',     qos_profile_sensor_data)
        self.publisher_ER    = self.create_publisher(PoseStamped, 'control/Elbow_right',    qos_profile_sensor_data)
        self.vr_connected_pub_ = self.create_publisher(Bool, 'info/vr_connected', 10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.tcp_sub_A        = self.create_subscription(PoseStamped, 'info/eef_left',        self.tcp_callback_A,    qos_profile_sensor_data)
        self.tcp_sub_B        = self.create_subscription(PoseStamped, 'info/eef_right',       self.tcp_callback_B,    qos_profile_sensor_data)
        self.system_state_sub = self.create_subscription(Int32,       'control/switch_state', self.system_state_cbk,  10)
        self.home_arm_caller  = self.create_client(Trigger, 'control/home_arm')

        if self.glove_mode:
            self.foot_key_state_sub = self.create_subscription(Int32, 'control/footkey', self.footkey_cbk, 10)
            self.foot_keyL      = False
            self.foot_keyM      = False
            self.foot_keyR      = False
            self.last_foot_keyR = False
        self.get_logger().info(f'Glove mode: {self.glove_mode}')

        # ── State ─────────────────────────────────────────────────────────────
        self.switch_state  = 0
        self.teleop_state  = True
        self.last_buttonX  = False

        self.pos_scale    = 1.1
        self.user_height  = 1.65
        self.torso_height = self.user_height * 0.66
        self.height_offset = self.torso_height - self.base_height

        self.max_x =  1.8
        self.min_x = -0.15
        self.max_y =  1.8
        self.min_y = -1.8
        self.max_z =  1.8
        self.min_z =  0.5

        # ── Networking ────────────────────────────────────────────────────────
        self.subnet = '.'.join(robot_ip.split('.')[:3]) + '.'
        self.vr_ip  = self.subnet + '124'
        self.get_logger().info(f'subnet: {self.subnet}  vr_ip: {self.vr_ip}')

        self.udp_ip    = '0.0.0.0'
        self.udp_portA = 9000
        self.udp_portB = 9001
        self.udp_fb_A  = 9002
        self.udp_fb_B  = 9003
        self.udp_portC = 9004

        self.discovery_ip          = '0.0.0.0'
        self.discovery_port        = 8888
        self.broadcast_name        = 'ApexHost'
        self.broadcast_model       = 'Orin_agx'
        self.broadcast_app_version = '251223'

        # UDP receive sockets (non-blocking)
        self.socka = self._make_udp_socket(self.udp_ip, self.udp_portA)
        self.sockb = self._make_udp_socket(self.udp_ip, self.udp_portB)
        self.sockc = self._make_udp_socket(self.udp_ip, self.udp_portC)

        # Discovery socket
        self.sock_discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_discovery.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock_discovery.bind((self.discovery_ip, self.discovery_port))
        except Exception as e:
            self.get_logger().warn(f'Could not bind discovery socket: {e}')
        self.sock_discovery.setblocking(False)

        # Broadcast socket
        self.sock_bcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_bcast.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # EEF feedback sockets
        self.sock_fb_A = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_fb_A.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock_fb_B = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_fb_B.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── TCP session manager ───────────────────────────────────────────────
        # threading.Event: set = headset connected, clear = disconnected
        self._tcp_connected = threading.Event()
        self._tcp_thread = threading.Thread(
            target=self._tcp_session_manager,
            args=(tcp_port, tcp_heartbeat_timeout),
            daemon=True,
            name='tcp_session_manager',
        )
        self._tcp_thread.start()
        self.get_logger().info(f'TCP session manager listening on port {tcp_port}')

        # ── Timers ────────────────────────────────────────────────────────────
        # Single timer polls all three UDP sockets at 1 ms (1 kHz)
        self.timer              = self.create_timer(0.001, self._poll_udp)
        self.discovery_timer    = self.create_timer(1.0,   self.listen_discovery)
        self.broadcast_timer    = self.create_timer(1.0,   self.send_broadcast)
        self.vr_connected_timer = self.create_timer(1.0,   self._pub_vr_connected)

        self.get_logger().info(
            f'Listening UDP on {self.udp_ip}:{self.udp_portA}, '
            f'{self.udp_portB}, {self.udp_portC}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_udp_socket(self, ip: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))
        sock.setblocking(False)
        return sock

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # ── TCP session manager (background thread) ───────────────────────────────

    def _tcp_session_manager(self, port: int, heartbeat_timeout: float):
        """
        Listens for one TCP connection at a time (blocking accept).
        - On connect  → sets _tcp_connected, logs info, updates vr_ip
        - Heartbeat   → expects any data within heartbeat_timeout seconds; echoes it back
        - On timeout  → WARN, clear _tcp_connected, wait for next connect
        - On graceful close / OS error → same as timeout
        """
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', port))
        server_sock.listen(1)

        while True:
            conn = None
            try:
                self.get_logger().info(f'[TCP] Waiting for headset on port {port}...')
                conn, addr = server_sock.accept()
                conn.settimeout(heartbeat_timeout)

                self._tcp_connected.set()
                self.get_logger().info(f'[TCP] Headset connected from {addr[0]}:{addr[1]}')

                # Update vr_ip to the connecting client's address
                if addr[0] != self.vr_ip:
                    self.get_logger().info(
                        f'[TCP] Updated vr_ip: {self.vr_ip} → {addr[0]}'
                    )
                    self.vr_ip = addr[0]

                # Heartbeat loop
                while True:
                    try:
                        data = conn.recv(256)
                        print(f'[TCP] Received heartbeat data: {data}')
                        if not data:
                            raise ConnectionResetError('remote closed connection')
                        conn.sendall(data)   # echo back
                    except socket.timeout:
                        self.get_logger().warn(
                            f'[TCP] Heartbeat timeout ({heartbeat_timeout:.1f}s) — '
                            f'headset {addr[0]} considered disconnected'
                        )
                        break
                    except (ConnectionResetError, OSError) as e:
                        self.get_logger().warn(f'[TCP] Connection lost: {e}')
                        break

            except Exception as e:
                self.get_logger().error(f'[TCP] Session manager error: {e}')

            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
                self._tcp_connected.clear()
                self.get_logger().warn(
                    '[TCP] Headset disconnected — UDP processing suspended'
                )

    # ── UDP polling ───────────────────────────────────────────────────────────

    def _pub_vr_connected(self):
        self.vr_connected_pub_.publish(Bool(data=self._tcp_connected.is_set()))

    def _poll_udp(self):
        """Single 1 kHz timer callback polling all three UDP sockets."""
        self.listen_udp_A()
        self.listen_udp_B()
        self.listen_udp_C()

    # ── Footkey / system-state callbacks ──────────────────────────────────────

    def footkey_cbk(self, msg):
        self.get_logger().info(f'Footkey state: {msg.data}')
        case = msg.data
        self.foot_keyL = (case == 1)
        self.foot_keyM = (case == 2)
        self.foot_keyR = (case == 3)

        if self.foot_keyR and not self.last_foot_keyR:
            if self.home_arm_caller.service_is_ready():
                future = self.home_arm_caller.call_async(Trigger.Request())
                future.add_done_callback(self.response_callback)
        self.last_foot_keyR = self.foot_keyR

    def system_state_cbk(self, msg):
        self.switch_state = msg.data
        self.teleop_state = self.switch_state in (0, 1)

    # ── UDP receive ───────────────────────────────────────────────────────────

    def listen_udp_A(self):
        if not self._tcp_connected.is_set():
            return
        try:
            data, _ = self.socka.recvfrom(1024)
            pose_data = json.loads(data.decode('utf-8'))

            x = self._clamp(pose_data['px'], self.min_x, self.max_x)
            y = self._clamp(pose_data['py'], self.min_y, self.max_y)
            z = self._clamp(pose_data['pz'] + self.height_offset, self.min_z, self.max_z)

            pose_msg = PoseStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'tracking_base_link'
            pose_msg.pose.position.x    = x * self.pos_scale
            pose_msg.pose.position.y    = y * self.pos_scale
            pose_msg.pose.position.z    = z * self.pos_scale
            pose_msg.pose.orientation.x = pose_data['qx']
            pose_msg.pose.orientation.y = pose_data['qy']
            pose_msg.pose.orientation.z = pose_data['qz']
            pose_msg.pose.orientation.w = pose_data['qw']

            grip_msg    = self.foot_keyL if self.glove_mode else pose_data['grip']
            trigger_msg = pose_data['triggerValue']

            self.publisher_A.publish(pose_msg)
            self.trigger_pose_A.publish(Bool(data=grip_msg))
            self.trigger_value_A.publish(Float32(data=trigger_msg))

            # Button X edge detection (reserved for future action)
            buttonX = pose_data['buttonX']
            if buttonX and not self.last_buttonX:
                pass
            self.last_buttonX = buttonX

        except BlockingIOError:
            pass

    def listen_udp_B(self):
        if not self._tcp_connected.is_set():
            return
        try:
            data, _ = self.sockb.recvfrom(1024)
            pose_data = json.loads(data.decode('utf-8'))

            x = self._clamp(pose_data['px'], self.min_x, self.max_x)
            y = self._clamp(pose_data['py'], self.min_y, self.max_y)
            z = self._clamp(pose_data['pz'] + self.height_offset, self.min_z, self.max_z)

            pose_msg = PoseStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'tracking_base_link'
            pose_msg.pose.position.x    = x * self.pos_scale
            pose_msg.pose.position.y    = y * self.pos_scale
            pose_msg.pose.position.z    = z * self.pos_scale
            pose_msg.pose.orientation.x = pose_data['qx']
            pose_msg.pose.orientation.y = pose_data['qy']
            pose_msg.pose.orientation.z = pose_data['qz']
            pose_msg.pose.orientation.w = pose_data['qw']

            grip_msg    = self.foot_keyL if self.glove_mode else pose_data['grip']
            trigger_msg = pose_data['triggerValue']

            self.publisher_B.publish(pose_msg)
            self.trigger_pose_B.publish(Bool(data=grip_msg))
            self.trigger_value_B.publish(Float32(data=trigger_msg))

        except BlockingIOError:
            pass

    def listen_udp_C(self):
        if not self._tcp_connected.is_set():
            return
        try:
            data, _ = self.sockc.recvfrom(1024)
            pose_data = json.loads(data.decode('utf-8'))

            elbow_L = pose_data['left_elbow']
            elbow_R = pose_data['right_elbow']

            # Update user height from headset
            user_height = pose_data.get('user_height')
            if user_height:
                self.user_height   = user_height
                self.torso_height  = 0.66 * self.user_height
                self.height_offset = self.torso_height - self.base_height

            elbow_L['pz'] += self.height_offset
            elbow_R['pz'] += self.height_offset

            def make_pose(e):
                msg = PoseStamped()
                msg.header.stamp        = self.get_clock().now().to_msg()
                msg.header.frame_id     = 'tracking_base_link'
                msg.pose.position.x     = e['px'] * self.pos_scale
                msg.pose.position.y     = e['py'] * self.pos_scale
                msg.pose.position.z     = e['pz'] * self.pos_scale
                msg.pose.orientation.x  = e['qx']
                msg.pose.orientation.y  = e['qy']
                msg.pose.orientation.z  = e['qz']
                msg.pose.orientation.w  = e['qw']
                return msg

            self.publisher_EL.publish(make_pose(elbow_L))
            self.publisher_ER.publish(make_pose(elbow_R))

        except BlockingIOError:
            pass

    # ── Discovery / broadcast ─────────────────────────────────────────────────

    def listen_discovery(self):
        """Update vr_ip from headset broadcast packets (fallback when TCP unused)."""
        try:
            data, addr = self.sock_discovery.recvfrom(4096)
            try:
                obj = json.loads(data.decode('utf-8', errors='replace'))
            except Exception:
                return
            if obj.get('name') == 'ApexHeadset':
                new_ip = addr[0]
                if new_ip != self.vr_ip:
                    self.get_logger().info(
                        f'[Discovery] Updated vr_ip: {self.vr_ip} → {new_ip}'
                    )
                    self.vr_ip = new_ip
        except BlockingIOError:
            pass

    def send_broadcast(self):
        """Broadcast presence so headsets can discover this node."""
        payload = {
            'name':        self.broadcast_name,
            'model':       self.broadcast_model,
            'app_version': self.broadcast_app_version,
        }
        try:
            bcast_addr = self.subnet + '255'
            self.sock_bcast.sendto(
                json.dumps(payload).encode('utf-8'),
                (bcast_addr, self.discovery_port)
            )
        except Exception as e:
            self.get_logger().debug(f'Broadcast failed: {e}')

    # ── EEF feedback (UDP back to headset) ───────────────────────────────────

    def tcp_callback_A(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'tracking_base_link', 'base_link', self.get_clock().now()
            )
            pose_out = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            feedback = PoseGripMsg(
                px=pose_out.position.x / self.pos_scale,
                py=pose_out.position.y / self.pos_scale,
                pz=pose_out.position.z / self.pos_scale - self.height_offset,
                qx=pose_out.orientation.x,
                qy=pose_out.orientation.y,
                qz=pose_out.orientation.z,
                qw=pose_out.orientation.w,
            )
            self.sock_fb_A.sendto(feedback.pack(), (self.vr_ip, self.udp_fb_A))
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def tcp_callback_B(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'tracking_base_link', 'base_link', self.get_clock().now()
            )
            pose_out = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            feedback = PoseGripMsg(
                px=pose_out.position.x / self.pos_scale,
                py=pose_out.position.y / self.pos_scale,
                pz=pose_out.position.z / self.pos_scale - self.height_offset,
                qx=pose_out.orientation.x,
                qy=pose_out.orientation.y,
                qz=pose_out.orientation.z,
                qw=pose_out.orientation.w,
            )
            self.sock_fb_B.sendto(feedback.pack(), (self.vr_ip, self.udp_fb_B))
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    # ── Misc ──────────────────────────────────────────────────────────────────

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'home_arm result: {response.success} — {response.message}'
            )
        except Exception as e:
            self.get_logger().error(f'home_arm service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UDPPoseReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
