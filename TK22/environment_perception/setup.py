from setuptools import find_packages, setup

package_name = 'environment_perception'

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
    maintainer='fortis_dev',
    maintainer_email='wael.mohammed@tuni.fi',
    description='ROS2 package for perceiving the environment. It includes audio analyser and visual quality assessment',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'audio_analyser = environment_perception.audio_analyser:main',
            'video_quality_assessment = environment_perception.visual_quality_assessment:main',
        ],
    },
)
