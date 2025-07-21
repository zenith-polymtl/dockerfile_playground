from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the .msg files for message generation
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg'))
    ],
    install_requires=['setuptools', 'mission_interfaces'],
    zip_safe=True,
    maintainer='colin',
    maintainer_email='colin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'state = mission.state_node_min:main',
            'approach = mission.approach:main',
            'vision = mission.machine_vision:main',
            'valve = mission.valve_control:main',
            'winch = mission.winch_control:main',
            'control = mission.manual_control:main',
            'water = mission.water_measure:main',
            'camera = mission.cam_pub:main',
        ],
    },
)
