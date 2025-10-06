from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from setuptools.command.install import install
from setuptools.command.develop import develop
import os
from glob import glob
from std_msgs.msg import String, Int32, Float32
from jinja2 import Environment, FileSystemLoader
import json

package_name = 'drone_interfaces'

types = {
    "Int32": Int32,
    "String": String,
    "Float32": Float32,
}

def generate_files():
    try:
        package_dir = os.path.dirname(os.path.abspath(__file__))
        control_interface_dir = os.path.join(package_dir, 'drone_interfaces', 'control_interface')
        
        config_path = os.path.join(control_interface_dir, 'polar.json')
        template_dir = os.path.join(control_interface_dir, 'templates')
        
        node_output_path = os.path.join(package_dir, 'drone_interfaces', 'web_manual_control_node.py')
        html_output_path = os.path.join(control_interface_dir, 'index.html')
        
        with open(config_path, 'r') as f:
            config = json.load(f)

        env = Environment(loader=FileSystemLoader(template_dir))
        
        node_template = env.get_template('template.py.j2')
        publishers = set()
        subscribers = set()
        
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

        html_template = env.get_template('template.html.j2')
        with open(html_output_path, 'w') as f:
            f.write(html_template.render(
                config=config,
                sidebar=config['sidebar'],
                control_sections=config['control_sections']
            ))

        static_src = os.path.join(template_dir, 'static')
        static_dest = os.path.join(control_interface_dir, 'static')
        
        if os.path.exists(static_src):
            if os.path.exists(static_dest):
                shutil.rmtree(static_dest)
            shutil.copytree(static_src, static_dest)

        return True
        
    except Exception as e:
        raise

class CustomBuildPy(build_py):
    def run(self):
        if generate_files():
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
    def run(self):
        generate_files()
        install.run(self)

class CustomDevelop(develop):
    def run(self):
        generate_files()
        develop.run(self)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'drone_interfaces/other_components']),
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
         glob('drone_interfaces/control_interface/*.html*')),
        (os.path.join('share', package_name, 'control_interface', 'static'), 
         glob('drone_interfaces/control_interface/static/*')),
    ],
    install_requires=[
        'setuptools',
        'Jinja2>=3.0.0',
        'drone_interfaces'
    ],
    zip_safe=False,
    cmdclass={
        'build_py': CustomBuildPy,
        'install': CustomInstall,
        'develop': CustomDevelop,
    },
    maintainer='colin',
    maintainer_email='colin@todo.todo',
    description='Drone control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = drone_interfaces.web_manual_control_node:main',
        ],
    },
)
