import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # ---------------------------------------------------------
    # 1. 🤖 相机 1：左腕相机 (Left Wrist)
    # ---------------------------------------------------------
    cam_left_wrist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            'camera_name': 'cam_left_wrist',
            'serial_no': '"260322271552"',
            'depth_module.color_profile': '640x480x30',  # D405 彩色分辨率规范参数
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_rgbd': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'false',
            'enable_color': 'true',
            'depth_module.enable_auto_exposure': 'false',
            'depth_module.exposure': '60000',
        }.items()
    )

    # ---------------------------------------------------------
    # 2. 🤖 相机 2：右腕相机 (Right Wrist)
    # ---------------------------------------------------------
    cam_right_wrist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            'camera_name': 'cam_right_wrist',
            'serial_no': '"260322279927"',
            'depth_module.color_profile': '640x480x30',  # D405 彩色分辨率规范参数
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_rgbd': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'false',
            'enable_color': 'true',
            'depth_module.enable_auto_exposure': 'false',
            'depth_module.exposure': '60000',
        }.items()
    )

    # ---------------------------------------------------------
    # 3. 🎥 相机 3：头部相机 (Head Cam)
    # ---------------------------------------------------------
    cam_head_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            'camera_name': 'cam_head',
            'serial_no': '"042222070564"',
            'rgb_camera.color_profile': '1280x720x30',
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_rgbd': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'false',
        }.items()
    )

    # =========================================================
    # 📦 4. 图像压缩节点 (image_transport republish)
    # 加 3 秒延时，等待相机驱动完全拉起后再启动转码，防止死锁
    # =========================================================
    
    # 左手压缩转码节点
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
        cam_left_wrist_launch,
        cam_right_wrist_launch,
        cam_head_launch,
        delayed_republish,  # 👈 自动启动 3 路压缩转码
    ])