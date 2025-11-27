from setuptools import find_packages, setup

package_name = 'exodus_mission_manager'

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
    description='ROS 2 package for high-level mission planning and execution management.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = exodus_mission_manager.mission_manager_node:main',
        ],
    },
)