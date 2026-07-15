from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # 方法1: 动态查找包路径 (类似ROS1的$(find package))
    # 查找node_manager包的安装路径
    username = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.environ.get('LOGNAME')
    storage_root = os.environ.get('BAG_STORAGE_ROOT', os.path.join('/media', username, 'BAG_STORAGE'))
    #storage_root = os.environ.get('BAG_STORAGE_ROOT', os.path.join('/home/marvin/KernelMind_Apex/BAG_STORAGE/'))
    # record_storage_path = os.path.abspath(rel_record_storage_path)
    record_storage_path = os.path.join(storage_root, 'recorded_bags')
    username = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.environ.get('LOGNAME') 
    storage_root = os.environ.get('BAG_STORAGE_ROOT', os.path.join('/media', username, 'BAG_STORAGE'))
    #storage_root = os.environ.get('BAG_STORAGE_ROOT', os.path.join('/home/marvin/KernelMind_Apex/BAG_STORAGE/'))
    # record_storage_path = os.path.abspath(rel_record_storage_path)
    record_storage_path = os.path.join(storage_root, 'recorded_bags')
    # print(record_storage_path)
    rel_playback_path = "src1/data/recorded_bags/"
    # playback_path = os.path.abspath(rel_playback_path)
    playback_path = os.path.join(storage_root, 'recorded_bags')
    return LaunchDescription([
               
        # playback_service,
        Node(
            package='bag_playback_nodes_py',
            executable='playback_service',
            name='playback_service',
            parameters=[{
                'bag_directory': playback_path,
                
            }]
        ),
        # playback_node,
        Node(
            package='bag_playback_nodes_py',
            executable='rd_playback_node',
            name='playback_node',
            output='screen',
        ),
        # recorder_node,
        Node(
            package='bag_recorder_nodes_py',
            executable='data_bag_recorder',
            name='data_bag_recorder',
            parameters=[{
                'record_bags_storage_dir': record_storage_path,
                'warning_light_enabled': True,
                'warning_light_port': os.environ.get('WARNING_LIGHT_PORT', 'auto'),
                'warning_light_baud': int(os.environ.get('WARNING_LIGHT_BAUD', '9600')),
                'warning_light_recording_color': os.environ.get('WARNING_LIGHT_RECORDING_COLOR', 'green'),
                'warning_light_stopped_color': os.environ.get('WARNING_LIGHT_STOPPED_COLOR', 'red'),
            
                # 可以在这里添加其他参数，或者同时使用config文件
            }],
            arguments=[
                "--headless"
            ]
        ),
    ])
