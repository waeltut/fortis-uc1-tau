from setuptools import find_packages, setup

package_name = 'costmaps_manager'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'audio_cost_manager = costmaps_manager.audio_cost_manager:main',
            'human_cost_manager = costmaps_manager.human_cost_manager:main',
            'video_cost_manager = costmaps_manager.video_cost_manager:main',
        ],
    },
)
