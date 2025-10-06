from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from setuptools.command.install import install
from setuptools.command.develop import develop
import os
from glob import glob
from std_msgs.msg import String, Int32, Float32
from jinja2 import Environment, FileSystemLoader
import json
import shutil

package_name = 'drone_interfaces'

types = {
    "Int32": Int32,
    "String": String,
    "Float32": Float32,
}

def sanitize_name(s: str) -> str:
    return "".join(c for c in s if c.isalnum() or c == "_")

def generate_nodes_and_html():
    package_dir = os.path.dirname(os.path.abspath(__file__))
    ci_dir = os.path.join(package_dir, 'drone_interfaces', 'control_interface')
    template_dir = os.path.join(ci_dir, 'templates')
    node_template = Environment(loader=FileSystemLoader(template_dir)).get_template('template.py.j2')
    html_template = Environment(loader=FileSystemLoader(template_dir)).get_template('template.html.j2')
    
    # Collect all JSON files in control_interface
    configs = []
    for fname in os.listdir(ci_dir):
        if fname.endswith('.json'):
            configs.append(fname)
    
    entry_nodes = []
    for cfgname in configs:
        cfg_path = os.path.join(ci_dir, cfgname)
        with open(cfg_path, 'r') as f:
            config = json.load(f)
        
        title = config.get('title', os.path.splitext(cfgname)[0])
        safe_name = sanitize_name(title)
        module_name = f"{safe_name}"
        py_path = os.path.join(package_dir, 'drone_interfaces', f"{module_name}.py")
        html_path = os.path.join(ci_dir, f"index.html")
        
        publishers = set()
        subscribers = set()
        for section in config.get('control_sections', []):
            for key in ['radio', 'buttons', 'inputs']:
                if key in section:
                    for item in section[key]:
                        publisher = (item['command'], types[item['topicType']], item['qosProfile'])
                        publishers.add(publisher)
            if 'sensors' in section:
                for sensor in section['sensors']:
                    subscriber = (sensor['command'], types[sensor['topicType']], sensor['qosProfile'])
                    subscribers.add(subscriber)
        # sidebar publishers
        for button in config.get('sidebar', {}).get('buttons', []):
            publisher_side = (button['command'], types[button['topicType']], button['qosProfile'])
            publishers.add(publisher_side)
        
        with open(py_path, 'w') as f:
            f.write(node_template.render(
                publishers=publishers,
                subscribers=subscribers,
                config=config,
                module_name=module_name,
                node_name=safe_name
            ))
        
        # --- Render HTML interface ---
        with open(html_path, 'w') as f:
            f.write(html_template.render(
                config=config,
                sidebar=config.get('sidebar', {}),
                control_sections=config.get('control_sections', [])
            ))
        
        # Copy static if exists
        static_src = os.path.join(template_dir, 'static')
        static_dst = os.path.join(ci_dir, f"static")
        if os.path.exists(static_src):
            if os.path.exists(static_dst):
                shutil.rmtree(static_dst)
            shutil.copytree(static_src, static_dst)
        
        # Define entry point for this generated node
        entry_nodes.append(f"{module_name} = drone_interfaces.{module_name}:main")
        
    return entry_nodes

class CustomBuildPy(build_py):
    def run(self):
        entry_nodes = generate_nodes_and_html()
        # Ensure generated files are included
        if not hasattr(self, 'distribution'):
            self.distribution = self.get_finalized_command('install').distribution
        pkg_data = getattr(self.distribution, 'package_data', {})
        pkg_data.setdefault(package_name, []).extend([
            "*.py",  # include generated .py modules
            "control_interface/*.html",
            "control_interface/static*/*"
        ])
        self.distribution.package_data = pkg_data
        
        build_py.run(self)

class CustomInstall(install):
    def run(self):
        generate_nodes_and_html()
        install.run(self)

class CustomDevelop(develop):
    def run(self):
        generate_nodes_and_html()
        develop.run(self)

# We call it here so entry_points is ready at setup time
entry_nodes = generate_nodes_and_html()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'drone_interfaces/other_components']),
    package_data={
        package_name: [
            "*.py",
            'control_interface/*.html',
            'control_interface/static*/*',
            'control_interface/templates/*',
            'control_interface/*.json'
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
    description='Drone control package (multi‑json)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': entry_nodes,
    },
)
