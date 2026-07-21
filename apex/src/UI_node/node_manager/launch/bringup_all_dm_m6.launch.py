from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction, ExecuteProcess

import os

from ament_index_python.packages import get_package_share_directory


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

    # 在 return LaunchDescription([...]) 中添加
    reset_gripper = TimerAction(
        period=5.0,  # 等5秒等所有节点就绪
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/control/reset_grippers', 'std_srvs/srv/Trigger', '{}'],
                output='screen'
            )
        ]
    )

    # ----------------------
    # Includes
    # ----------------------
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('marvin_teleop'),
                'launch',
                'teleop_m6.launch.py'
            )
        ),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )

    dm_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dm_gripper_py'),
                'launch',
                'dm_gripper.launch.py'
            )
        )
    )

    # quadcam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('gmsl_quadcam'),
    #             'launch',
    #             'quad_csi_webrtc.launch.py'
    #         )
    #     )
    # )

    ui_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('node_manager'),
                'launch',
                'ui_nodes.launch.py'
            )
        )
    )

    all_topic_log_recorder_node = Node(
        package='log_recorder_nodes_py',
        executable='all_topic_log_recorder',
        name='all_topic_log_recorder',
        output='screen'
    )

    # 相机 1：左腕相机 (Left Wrist)
    cam_left_wrist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'cam_left_wrist',
            'serial_no': '"260322271552"', # 替换为实际的序列号
            'rgb_camera.color_profile': '640x480x30',
            'enable_depth': 'false',   # 对应参数：enable_depth
            'enable_infra1': 'false',  # 对应参数：enable_infra1
            'enable_infra2': 'false',  # 对应参数：enable_infra2
            'enable_rgbd': 'false',    # 对应参数：enable_rgbd
            'enable_gyro': 'false',    # 对应参数：enable_gyro
            'enable_accel': 'false',   # 对应参数：enable_accel
            'pointcloud.enable': 'false', # 对应参数：pointcloud.enable
            'enable_color': 'true',
            'depth_module.enable_auto_exposure': 'false',
            'depth_module.exposure': '60000',
        }.items()
    )

    cam_right_wrist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'cam_right_wrist',
            'serial_no': '"260322279927"', # 替换为实际的序列号
            'rgb_camera.color_profile': '640x480x30',
            'enable_depth': 'false',   # 对应参数：enable_depth
            'enable_infra1': 'false',  # 对应参数：enable_infra1
            'enable_infra2': 'false',  # 对应参数：enable_infra2
            'enable_rgbd': 'false',    # 对应参数：enable_rgbd
            'enable_gyro': 'false',    # 对应参数：enable_gyro
            'enable_accel': 'false',   # 对应参数：enable_accel
            'pointcloud.enable': 'false', # 对应参数：pointcloud.enable
            'enable_color': 'true',
            'depth_module.enable_auto_exposure': 'false',
            'depth_module.exposure': '60000',
        }.items()
    )

    cam_head_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'cam_head',
            'serial_no': '"042222070564"', # 替换为实际的序列号
            'rgb_camera.color_profile': '1280x720x30',
            'enable_color': 'true',
            'enable_depth': 'false',   # 对应参数：enable_depth
            'enable_infra1': 'false',  # 对应参数：enable_infra1
            'enable_infra2': 'false',  # 对应参数：enable_infra2
            'enable_rgbd': 'false',    # 对应参数：enable_rgbd
            'enable_gyro': 'false',    # 对应参数：enable_gyro
            'enable_accel': 'false',   # 对应参数：enable_accel
            'pointcloud.enable': 'false', # 对应参数：pointcloud.enable,
        }.items()
    )

    republish_left = Node(
        package='image_transport',
        executable='republish',
        name='republish_cam_left',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/cam_left_wrist/color/image_raw'),
            ('out/compressed', '/camera/cam_left_wrist/color/image_raw/compressed')
        ]
    )

    # 右手压缩转码节点
    republish_right = Node(
        package='image_transport',
        executable='republish',
        name='republish_cam_right',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/cam_right_wrist/color/image_raw'),
            ('out/compressed', '/camera/cam_right_wrist/color/image_raw/compressed')
        ]
    )

    # 头部压缩转码节点
    republish_head = Node(
        package='image_transport',
        executable='republish',
        name='republish_cam_head',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/cam_head/color/image_raw'),
            ('out/compressed', '/camera/cam_head/color/image_raw/compressed')
        ]
    )

    # 延迟 3.0 秒启动压缩节点，确保原始图像流稳定输出
    delayed_republish = TimerAction(
        period=3.0,
        actions=[republish_left, republish_right, republish_head]
    )

    return LaunchDescription([
        reset_gripper,
        declare_use_rviz,
        teleop_launch,
        dm_gripper_launch,
        # quadcam_launch,
        ui_nodes_launch,
        # all_topic_log_recorder_node,
        cam_left_wrist_launch,
        cam_right_wrist_launch,
        cam_head_launch,
        delayed_republish,
    ])

