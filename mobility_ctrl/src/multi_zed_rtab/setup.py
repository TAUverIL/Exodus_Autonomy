from setuptools import setup
import os
from glob import glob

package_name = 'multi_zed_rtab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install ALL config files (params, yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 2. Install ALL launch files (nav2, cameras, everything)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tauver',
    maintainer_email='tauver@todo.todo',
    description='Python-based ROS 2 package for multiple ZED cameras and RTAB-Map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cost_generator_node = multi_zed_rtab.cost_generator_node:main',
        ],
    },
)