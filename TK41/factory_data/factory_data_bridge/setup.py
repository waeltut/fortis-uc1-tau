from setuptools import setup
from glob import glob

package_name = 'factory_data_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Factory Toolkit Maintainer',
    maintainer_email='maintainer@example.com',
    description='Fetch factory JSON through a ROS 2 service.',
    license='MIT',
    entry_points={'console_scripts': ['factory_data_service = factory_data_bridge.service_node:main']},
)
