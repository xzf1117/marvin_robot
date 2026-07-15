from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
import pwd
# 这两条是运行其他launch必须的
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# 运行终端的命令需要这个包
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 方法1: 动态查找包路径 (类似ROS1的$(find package))
    # 查找node_manager包的安装路径
    node_manager_share_dir = get_package_share_directory('node_manager')
    rel_record_storage_path = "src1/data/recorded_bags/"
    # 动态获取当前用户名，并允许通过环境变量覆盖根目录
    username = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.environ.get('LOGNAME') or pwd.getpwuid(os.getuid()).pw_name
    storage_root = os.environ.get('BAG_STORAGE_ROOT', os.path.join('/media', username, 'BAG_STORAGE'))
    # record_storage_path = os.path.abspath(rel_record_storage_path)
    record_storage_path = os.path.join(storage_root, 'recorded_bags')
    # print(record_storage_path)
    rel_playback_path = "src1/data/recorded_bags/"
    # playback_path = os.path.abspath(rel_playback_path)
    playback_path = os.path.join(storage_root, 'recorded_bags')

    rel_convert_bags_path = "src1/data/recorded_bags/"
    # convert_bags_path = os.path.abspath(rel_convert_bags_path)
    convert_bags_path = os.path.join(storage_root, 'recorded_bags')

    rel_converted_bags_path = "src1/data/converted_bags/"
    # converted_bags_path = os.path.abspath(rel_converted_bags_path)
    converted_bags_path = os.path.join(storage_root, 'converted_bags')

    # usb_cam params file (equivalent to: --params-file ./install/usb_cam/share/usb_cam/config/cam0.yaml)
    usb_cam_params_file = os.path.join(
        get_package_share_directory('usb_cam'), 'config', 'cam0.yaml'
    )

    # 可选：获取配置文件路径（如果还想保留YAML配置其他参数）
    config_file = os.path.join(node_manager_share_dir, 'config', 'config.yaml')

    rosbridge_server_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
            )
        ),
        # launch_arguments=[('param1', 'foo')],
    )

    npm_proc = ExecuteProcess(
        cmd=[
            # 'gnome-terminal', '--', 'bash', '-c', 'npm run dev; exec bash'
            'npm', 'run', 'preview'
            # 'npx', 'serve', '-s', 'dist', '--listen', 'tcp://0.0.0.0:4173'
        ],
        cwd='UI/',  # ← 这里写你npm项目目录
        output='screen'
    )

    return LaunchDescription([
        # robot_mode_service - 机器人模式服务
        Node(
            package='robot_setting',
            executable='robot_mode_service.py',
            name='robot_mode_service',
            output='screen'
        ),
        
        # simple_bag_recorder - 使用动态路径
        Node(
            package='bag_recorder_nodes_py',
            executable='simple_bag_recorder',
            name='simple_bag_recorder',
            parameters=[{
                'record_bags_storage_dir': record_storage_path,
                # 可以在这里添加其他参数，或者同时使用config文件
            }]
            # 或者同时使用: parameters=[config_file, {'record_bags_storage_dir': record_storage_path}]
        ),
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
            executable='playback_node',
            name='playback_node'
        ),

        # bag_converter_service,
        Node(
            package='bag_converter_nodes_py',
            executable='bag_converter_service',
            name='bag_converter_service',
            parameters=[{
                'bags_directory': convert_bags_path,
                'converted_bags_directory': converted_bags_path
                # 可以在这里添加其他参数，或者同时使用config文件
            }]
        ),
        # bag_converter_node,
        Node(
            package='bag_converter_nodes_py',
            executable='bag_converter_node',
            name='bag_converter_node',
            parameters=[{
                'converted_bags_directory': converted_bags_path
            }]
        ),
        
        # usb_cam node (equivalent to:
        # ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_0 --params-file ./install/usb_cam/share/usb_cam/config/cam0.yaml)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='/usb_cam_0',
            parameters=[usb_cam_params_file],
            output='screen'
        ),
        
        # web_video_server,
        Node(
           package='web_video_server',
           executable='web_video_server',
           name='web_video_server'
        ),

        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(
                     get_package_share_directory('marvin_ros_control'),
                     'launch',
                     'teleop_m3.launch.py'
                 )
             ),
         ),

        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(
                     get_package_share_directory('dm_gripper_py'),
                     'launch',
                     'dm_gripper.launch.py'
                 )
             ),
         ),
        npm_proc,
        rosbridge_server_launch, 
    ])
