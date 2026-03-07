import os
from glob import glob
from setuptools import setup

package_name = 'llm_robot_task_planner'

# Collect all jetrover_description data files (meshes, urdf)
jetrover_data_files = []
jetrover_base = 'urdf/jetrover_description'
for dirpath, dirnames, filenames in os.walk(jetrover_base):
    if filenames:
        install_dir = os.path.join('share', package_name, dirpath)
        file_paths = [os.path.join(dirpath, f) for f in filenames
                      if not f.endswith('.py') and f != 'COLCON_IGNORE'
                      and f != '__init__.py' and f != 'package.xml'
                      and f != 'setup.cfg']
        if file_paths:
            jetrover_data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*'))),
    ] + jetrover_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammad Shoaib',
    maintainer_email='shoaib6174@gmail.com',
    description='LLM-powered robot task planner with ROS 2 and Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = llm_robot_task_planner.odom_to_tf_node:main',
            'perception_node = llm_robot_task_planner.perception_node:main',
            'arm_controller = llm_robot_task_planner.arm_controller_node:main',
        ],
    },
)
