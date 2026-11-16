from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mur_test'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mur',
    maintainer_email='maintainer@example.com',
    description='Manually confirmed MiR navigation and fresh post-navigation MoveIt arm motion test',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'pose_height_relay = mur_test.pose_height_relay:main','mur_test = mur_test.test_node:main']},
)
