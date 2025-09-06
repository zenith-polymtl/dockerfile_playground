from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from setuptools.command.develop import develop
from setuptools.command.install import install
import os
from glob import glob
import json
from jinja2 import Environment, FileSystemLoader
import shutil

package_name = 'mission'

def generate_interface():
    """Generate the HTML interface from templates and config."""
    try:
        package_dir = os.path.dirname(os.path.abspath(__file__))
        control_interface_dir = os.path.join(package_dir, 'mission', 'control_interface')
        
        config_path = os.path.join(control_interface_dir, 'control_config.json')
        template_dir = os.path.join(control_interface_dir, 'templates')
        output_path = os.path.join(control_interface_dir, 'index.html')
        
        print(f"[DEBUG] Generating interface from: {config_path}")
        
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        env = Environment(loader=FileSystemLoader(template_dir))
        template = env.get_template('template.html.j2')
        html_output = template.render(
            sidebar=config['sidebar'],
            control_sections=config['control_sections'],
        )
        
        with open(output_path, 'w') as f:
            f.write(html_output)
        
        print(f"[DEBUG] Successfully generated: {output_path}")
        
        return output_path
        
    except Exception as e:
        print(f"[ERROR] Failed to generate interface: {str(e)}")
        raise

class CustomBuildPy(build_py):
    """Custom build command to ensure index.html is included."""
    def run(self):
        html_path = generate_interface()
        
        if html_path and os.path.exists(html_path):
            if not hasattr(self, 'distribution'):
                self.distribution = self.distribution or self.get_finalized_command('install').distribution
            self.distribution.package_data = getattr(self.distribution, 'package_data', {})
            self.distribution.package_data.setdefault(package_name, []).append(
                'control_interface/index.html'
            )
        
        build_py.run(self)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'control_interface/*',
            'control_interface/static/*',
            'control_interface/index.html'
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
        'mission_interfaces',
        'Jinja2>=3.0.0'
    ],
    zip_safe=False,
    cmdclass={
        'build_py': CustomBuildPy,
        'install': install,
        'develop': develop,
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
            'align_raw = mission.align_node_raw_baselink:main',
            'vision = mission.machine_vision:main',
            'control = mission.web_manual_control_node:main',
            'water = mission.water_measure:main',
            'camera = mission.cam_pub:main',
            'graph = mission.graph_node:main',
            'graph_baselink = mission.graph_baselink_node:main',
            'abort = mission.abort_brake_node:main',
            'targets = mission.target_publisher_node:main',
            'targets_baselink = mission.target_baselink_publisher_node:main',
            'valve = mission.valve_control:main'
        ],
    },
)