#!/usr/bin/env python3
import os
import threading

import mujoco
import mink
import numpy as np
from mink.exceptions import NoSolutionFound

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from marvin_msgs.msg import IKRequest, IKResult
from ament_index_python.packages import get_package_share_directory


package_path = get_package_share_directory('marvin_teleop')
xml = os.path.join(package_path, 'config', 'mjcf', 'teleop_scene_pro.xml')


class MarvinIKNode(Node):
    def __init__(self):
        super().__init__('marvin_ik_node')

        self.declare_parameter('enable_viewer', False)
        self.enable_viewer = self.get_parameter('enable_viewer').get_parameter_value().bool_value

        sensor_data_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.IKrequest_sub = self.create_subscription(
            IKRequest, 'control/ik_request', self.ik_request_callback, sensor_data_qos)
        self.IKResult_pub = self.create_publisher(IKResult, 'control/ik_result', sensor_data_qos)
        # self.mode_sub = self.create_subscription(
        #     Int32, 'control/switch_state', self.mode_callback, 10)

        self.mode = 0
        self.target_lock = threading.Lock()
        self.joint_damping = [2.0,2.0,1.0,1.0,1.0,0.5,0.5, 2.0,2.0,1.0,1.0,1.0,0.5,0.5]  # Higher damping for joints closer to the base

        # --- MuJoCo / mink setup ---
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.configuration = mink.Configuration(self.model)
        self.configuration.update_from_keyframe("home")
        self.model = self.configuration.model
        self.data = self.configuration.data

        hands = ["left_tool_site", "right_tool_site"]
        elbows = ["Link4_L", "Link4_R"]

        self.posture_task = mink.PostureTask(self.model, cost=0.05)
        self.posture_task.set_target_from_configuration(self.configuration)
        damping_task = mink.DampingTask(self.model, cost=self.joint_damping)
        kinetic_task = mink.KineticEnergyRegularizationTask(cost=1e-4)
        kinetic_task.set_dt(1.0 / 200.0)
        self.hand_tasks = [
            mink.FrameTask(
                frame_name=hand,
                frame_type="site",
                position_cost=1.0,
                orientation_cost=0.1,
            )
            for hand in hands
        ]

        # Run FK so oMf is valid before set_target_from_configuration
        mujoco.mj_kinematics(self.model, self.data)
        for hand_task in self.hand_tasks:
            hand_task.set_target_from_configuration(self.configuration)
        self.tasks = [self.posture_task, damping_task, kinetic_task] + self.hand_tasks

        collision_pairs = [
            (["Link4_L_collision", "Link4_R_collision"], ["base_link_collision"]),
            (["Link5_L_collision", "Link5_R_collision"], ["base_link_collision"]),
        ]
        self.limits = [
            mink.ConfigurationLimit(self.model),
            mink.CollisionAvoidanceLimit(
                model=self.model,
                geom_pairs=collision_pairs,
                minimum_distance_from_collisions=0.005,
                collision_detection_distance=0.15,
            ),
            mink.VelocityLimit(self.model, {
                "Joint1_L": 1.5*np.pi, "Joint2_L": 2.5*np.pi, "Joint3_L": 2.5*np.pi,
                "Joint4_L": 2.5*np.pi, "Joint5_L": 2.5*np.pi, "Joint6_L": 2.5*np.pi,
                "Joint1_R": 1.5*np.pi, "Joint2_R": 2.5*np.pi, "Joint3_R": 2.5*np.pi,
                "Joint4_R": 2.5*np.pi, "Joint5_R": 2.5*np.pi, "Joint6_R": 2.5*np.pi,
            }),
        ]

        self.solver = "daqp"
        self.dt = 1.0 / 200.0
        self._last_no_solution_log_time = None

        self.viewer = None
        if self.enable_viewer:
            self._start_viewer()

        self.timer = self.create_timer(self.dt, self.ik_step)
        status = 'with viewer' if self.viewer is not None else 'no viewer'
        self.get_logger().info(f'MarvinIKNode initialized at 200 Hz ({status})')

    def _start_viewer(self):
        try:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.get_logger().info('MuJoCo viewer enabled')
        except Exception as exc:
            self.viewer = None
            self.get_logger().error(f'Failed to start MuJoCo viewer: {exc}')

    def close_viewer(self):
        if self.viewer is None:
            return
        try:
            self.viewer.close()
        except Exception:
            pass
        self.viewer = None

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                       #
    # ------------------------------------------------------------------ #
    def ik_request_callback(self, msg: IKRequest):
        with self.target_lock:
            if not any(msg.available):
                self.mode = 0
                # Sync configuration to real joint state, then anchor hand targets
                self.configuration.update(np.array(msg.joint_state.position))
                mujoco.mj_kinematics(self.model, self.data)
                for hand_task in self.hand_tasks:
                    hand_task.set_target_from_configuration(self.configuration)

            # Override with requested targets
            if msg.available[0]:
                self.mode = 1
                self.hand_tasks[0].set_target(pose_to_se3(msg.left_hand_pose))
            if msg.available[1]:
                self.mode = 1
                self.hand_tasks[1].set_target(pose_to_se3(msg.right_hand_pose))

    # def mode_callback(self, msg: Int32):
    #     self.mode = msg.data

    # ------------------------------------------------------------------ #
    #  IK step (called by timer)                                           #
    # ------------------------------------------------------------------ #
    def ik_step(self):
        with self.target_lock:
            try:
                vel = mink.solve_ik(
                    self.configuration, self.tasks, self.dt, self.solver,
                    damping=1e-1, limits=self.limits
                )
            except NoSolutionFound as exc:
                now = self.get_clock().now()
                if (
                    self._last_no_solution_log_time is None
                    or (now - self._last_no_solution_log_time).nanoseconds > 1_000_000_000
                ):
                    self.get_logger().warn(
                        f'IK solver failed for current target; holding previous command: {exc}'
                    )
                    self._last_no_solution_log_time = now
                # return
                vel = np.zeros(self.model.nv)
            self.configuration.integrate_inplace(vel, self.dt)

            if self.viewer is not None:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    self.viewer = None

        if self.mode == 1:
            ik_result_msg = IKResult()
            ik_result_msg.header.stamp = self.get_clock().now().to_msg()
            ik_result_msg.joint_state.position = self.configuration.q.tolist()
            self.IKResult_pub.publish(ik_result_msg)


# ------------------------------------------------------------------ #
#  Helpers                                                             #
# ------------------------------------------------------------------ #
def pose_to_se3(pose_msg: Pose) -> mink.SE3:
    pos = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    quat = np.array([pose_msg.orientation.w, pose_msg.orientation.x,
                     pose_msg.orientation.y, pose_msg.orientation.z])
    return mink.SE3(np.concatenate([quat, pos]))


# ------------------------------------------------------------------ #
#  Entry point                                                         #
# ------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = MarvinIKNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info('Shutting down...')
        node.close_viewer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
