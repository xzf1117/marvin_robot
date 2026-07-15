from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bag_playback_nodes_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanwen',
    maintainer_email='hanwen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'playback_node = bag_playback_nodes_py.playback_node:main',
            'rd_playback_node = bag_playback_nodes_py.playback_node_h:main',
            'playback_service = bag_playback_nodes_py.playback_service:main',
        ],
    },
)
