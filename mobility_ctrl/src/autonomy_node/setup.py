from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomy_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tauver',
    maintainer_email='lev.roibak@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_sub = autonomy_node.camera_subscriber:main',
            'detector_3cam = autonomy_node.detector_3cam:main',
            'grid_mapper = autonomy_node.grid_mapper_3cam:main',
            'odom_fusion = autonomy_node.odom_fusion:main',
            'path_planner = autonomy_node.path_planner:main',
        ],
    },
    
)
