from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from setuptools.command.develop import develop
from setuptools.command.install import install
import os
from glob import glob

package_name = 'mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'web_manual_control_node.py',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=[
        'setuptools',
        'mission_interfaces'
    ],
    zip_safe=False,
    maintainer='colin',
    maintainer_email='colin@todo.todo',
    description='Drone mission control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state = mission.state_node_min:main',
            'approach = mission.approach_node:main',
            'approach_raw = mission.approach_node_raw:main',
            'vision = mission.machine_vision:main',
            'water = mission.water_measure:main',
            'camera = mission.cam_pub:main',
            'graph = mission.graph_node:main',
            'abort = mission.abort_brake_node:main',
            'targets = mission.target_publisher_node:main',
            'valve = mission.valve_control:main'
        ],
    },
)