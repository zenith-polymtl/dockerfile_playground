from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from setuptools.command.develop import develop
from setuptools.command.install import install
import os
from glob import glob
import json
from jinja2 import Environment, FileSystemLoader
import shutil
from std_msgs.msg import String, Int32, Float32

package_name = 'mission'

types = {
    "Int32": Int32,
    "String": String,
    "Float32": Float32,
}

def generate_files():
    """Generate both node file and HTML interface."""
    try:
        package_dir = os.path.dirname(os.path.abspath(__file__))
        control_interface_dir = os.path.join(package_dir, 'mission', 'control_interface')
        
        # Path configuration
        config_path = os.path.join(control_interface_dir, 'control_config.json')
        template_dir = os.path.join(control_interface_dir, 'templates')
        
        # Output paths
        node_output_path = os.path.join(package_dir, 'mission', 'web_manual_control_node.py')
        html_output_path = os.path.join(control_interface_dir, 'index.html')
        
        print(f"[INFO] Generating files from config: {config_path}")

        # Load configuration
        with open(config_path, 'r') as f:
            config = json.load(f)

        # Generate Node File
        env = Environment(loader=FileSystemLoader(template_dir))
        
        # 1. Generate Node File
        node_template = env.get_template('template.py.j2')
        publishers = set()
        subscribers = set()
        
        # Collect publishers from all sections
        for section in config['control_sections']:
            if 'radio' in section:
                for radio in section['radio']:
                    publisher_radio = (
                        radio['command'],
                        types[radio['topicType']],
                        radio['qosProfile'],
                    )
                    publishers.add(publisher_radio)
            if 'buttons' in section:
                for button in section['buttons']:
                    publisher_button = (
                        button['command'],
                        types[button['topicType']],
                        button['qosProfile'],
                    )
                    publishers.add(publisher_button)
            if 'inputs' in section:
                for input in section['inputs']:
                    publisher_input = (
                        input['command'],
                        types[input['topicType']],
                        input['qosProfile'],
                    )
                    publishers.add(publisher_input)
            if 'sensors' in section:
                for sensor in section['sensors']:
                    subscriber = (
                        sensor['command'],
                        types[sensor['topicType']],
                        sensor['qosProfile'],
                    )
                    subscribers.add(subscriber)

        # Add sidebar publishers
        for button in config['sidebar']['buttons']:
            publisher_side = (
                button['command'],
                types[button['topicType']],
                button['qosProfile'],
            )
            publishers.add(publisher_side)
        
        with open(node_output_path, 'w') as f:
            f.write(node_template.render(
                publishers=publishers,
                subscribers=subscribers,
                config=config
            ))
        print(f"[INFO] Generated node file: {node_output_path}")

        # 2. Generate HTML Interface
        html_template = env.get_template('template.html.j2')
        with open(html_output_path, 'w') as f:
            f.write(html_template.render(
                sidebar=config['sidebar'],
                control_sections=config['control_sections']
            ))
        print(f"[INFO] Generated HTML interface: {html_output_path}")

        # 3. Copy static files if they exist
        static_src = os.path.join(template_dir, 'static')
        static_dest = os.path.join(control_interface_dir, 'static')
        
        if os.path.exists(static_src):
            if os.path.exists(static_dest):
                shutil.rmtree(static_dest)
            shutil.copytree(static_src, static_dest)
            print(f"[INFO] Copied static files to: {static_dest}")

        return True
        
    except Exception as e:
        print(f"[ERROR] Failed to generate files: {str(e)}")
        raise

class CustomBuildPy(build_py):
    """Custom build command to generate files before building."""
    def run(self):
        if generate_files():
            # Ensure generated files are included in package data
            if not hasattr(self, 'distribution'):
                self.distribution = self.get_finalized_command('install').distribution
            
            self.distribution.package_data = getattr(self.distribution, 'package_data', {})
            self.distribution.package_data.setdefault(package_name, []).extend([
                'web_manual_control_node.py',
                'control_interface/index.html',
                'control_interface/static/*'
            ])
        
        build_py.run(self)

class CustomInstall(install):
    """Custom install command to generate files before installation."""
    def run(self):
        generate_files()
        install.run(self)

class CustomDevelop(develop):
    """Custom develop command for development installations."""
    def run(self):
        generate_files()
        develop.run(self)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'web_manual_control_node.py',
            'control_interface/*.html',
            'control_interface/static/*',
            'control_interface/templates/*',
            'control_interface/control_config.json'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'control_interface'), 
         glob('mission/control_interface/*.html*')),
        (os.path.join('share', package_name, 'control_interface', 'static'), 
         glob('mission/control_interface/static/*')),
    ],
    install_requires=[
        'setuptools',
        'Jinja2>=3.0.0',
        'mission_interfaces'
    ],
    zip_safe=False,
    cmdclass={
        'build_py': CustomBuildPy,
        'install': CustomInstall,
        'develop': CustomDevelop,
    },
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
            'control = mission.web_manual_control_node:main',
            'water = mission.water_measure:main',
            'camera = mission.cam_pub:main',
            'graph = mission.graph_node:main',
            'abort = mission.abort_brake_node:main',
            'targets = mission.target_publisher_node:main',
            'valve = mission.valve_control:main'
        ],
    },
)