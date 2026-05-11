from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'slam_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
        ('share/' + package_name + '/config', ['config/mapper_params_localization.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/nav2_launch.py']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', ['maps/map3.yaml', 'maps/map3.pgm']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fortis_dev',
    maintainer_email='wael.mohammed@tuni.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_displayer = slam_config.map_displayer:main',
        ],
    },
)
