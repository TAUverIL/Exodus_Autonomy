import os
from glob import glob
from setuptools import setup

package_name = 'exodus_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))),
        # Install params files
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2user',
    maintainer_email='ros2user@todo.todo',
    description='Navigation configuration and launch files for Exodus Autonomy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)