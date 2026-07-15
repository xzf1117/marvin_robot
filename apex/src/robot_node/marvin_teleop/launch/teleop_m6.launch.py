from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ----------------------
    # Launch arguments
    # ----------------------
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # ----------------------
    # Config file
    # ----------------------
    control_config = os.path.join(
        get_package_share_directory('marvin_ros_control'),
        'config',
        'robot_param_m6.yaml'
    ),
    config = os.path.join(
        get_package_share_directory('marvin_teleop'),
        'config',
        'robot_param_m6.yaml'
    )

    # ----------------------
    # Nodes
    # ----------------------
    motion_node = Node(
        package='marvin_teleop',
        executable='IK_v2',
        name='motion_node',
        parameters=[
            config,
            control_config,
            {"config_file": 'ccs_m6.MvKDCfg'},
            {"urdf_file": 'marvin_CCS_m6.urdf'},
            {"srdf_file": 'marvin_robot.srdf'},
        ],
        output='screen',
    )

    ik_node = Node(
        package='marvin_teleop',
        executable='mink_ik_node.py',
        name='mink_ik_node',
        parameters=[config,{'enable_viewer': False}],
        output='screen',
    )

    vr_reader = Node(
        package='marvin_teleop',
        executable='controller_udp.py',
        name='VR_reader',
        parameters=[control_config,config],
        output='screen',
    )

    planner_node = Node(
        package='marvin_teleop',
        executable='planner_node',
        name='planner_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    joint_mux_node = Node(
        package='marvin_teleop',
        executable='joint_cmd_mux',
        name='joint_mux_node',
    )

    # ----------------------
    # Include ros_control bringup
    # ----------------------
    bringup_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('marvin_ros_control'),
                'launch',
                'bringup_control_m6.launch.py'
            )
        ),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )

    return LaunchDescription([
        declare_use_rviz,
        motion_node,
        ik_node,
        vr_reader,
        planner_node,
        bringup_control,
        joint_mux_node
    ])

