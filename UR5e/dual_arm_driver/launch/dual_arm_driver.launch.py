from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_arm_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fortis_dev',
    maintainer_email='todo@example.com',
    description='Headless dual UR5e pose and MoveIt interface',
    license='TODO',
    entry_points={
        'console_scripts': [
            'tcp_pose_publisher = '
            'dual_arm_driver.tcp_pose_publisher:main',

            'moveit_pose_commander = '
            'dual_arm_driver.moveit_pose_commander:main',
        ],
    },
)