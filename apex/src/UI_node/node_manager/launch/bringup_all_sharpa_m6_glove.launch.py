from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # ----------------------
    # Includes
    # ----------------------
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('marvin_teleop'),
                'launch',
                'teleop_m6_glove.launch.py'
            )
        ),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )

    quadcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gmsl_quadcam'),
                'launch',
                'quad_csi_webrtc.launch.py'
            )
        )
    )

   

    ui_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('node_manager'),
                'launch',
                'ui_nodes.launch.py'
            )
        )
    )

    return LaunchDescription([
        declare_use_rviz,
        teleop_launch,
        quadcam_launch,
        ui_nodes_launch,
    ])

