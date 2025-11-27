from setuptools import find_packages, setup

package_name = 'exodus_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Exodus',
    maintainer_email='team@exodus.com',
    description='ROS 2 package for multi-sensor data acquisition and 360 fused pointcloud generation.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = exodus_perception.perception_node:main',
        ],
    },
)