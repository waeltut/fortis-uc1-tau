from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'sensing_controller'

model_data_files = []
for root, dirs, files in os.walk("models"):     # Thank you Mr. GPT
    if files:  # only folders that contain files
        # Build target install path
        install_path = os.path.join('share', package_name, root)
        # Full paths to source files
        file_paths = [os.path.join(root, f) for f in files]
        model_data_files.append((install_path, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *model_data_files,
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
            'hearing = sensing_controller.hearing:main',
            'vision = sensing_controller.vision:main',
        ],
    },
)
