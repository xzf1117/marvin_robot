from setuptools import find_packages, setup

package_name = 'log_recorder_nodes_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanwen',
    maintainer_email='hanwen@todo.todo',
    description='Rolling all-topic log recorder node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'all_topic_log_recorder = '
            'log_recorder_nodes_py.all_topic_log_recorder:main',
        ],
    },
)
