from setuptools import find_packages, setup

package_name = 'mic_driver'

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
    description='microphone driver for ROS2. this package publishes raw audio data from a microphone.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mic_publisher = mic_driver.mic_publisher:main',
        ],
    },
)
