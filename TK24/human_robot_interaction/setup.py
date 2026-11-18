from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'human_robot_interaction'
"""model_data_files = []
for root, dirs, files in os.walk("stt_model"):     
    if files:  # only folders that contain files
        # Build target install path
        install_path = os.path.join('share', package_name, root)
        # Full paths to source files
        file_paths = [os.path.join(root, f) for f in files]
        model_data_files.append((install_path, file_paths))
"""
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'stt_model'), glob('human_robot_interaction/stt_model/*.pt')),
        (os.path.join('share', package_name, 'param_config'), glob('human_robot_interaction/param_config/*.yaml')),
        (os.path.join('share', package_name, 'context_config'), glob('human_robot_interaction/context_config/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fortis_dev',
    maintainer_email='wael.mohammed@tuni.fi',
    description='ROS2 package for analyzing noise levels and frequencies from microphone input.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #'visual_audio_mm = human_robot_interaction.visual_audio_mm:main',
            'audio_descriptor_processing_node = human_robot_interaction.audio_descriptor_processing_node:main', #descriptor node
            'audio_transcription_node = human_robot_interaction.audio_transcription_node:main', #transcription node
            
            'visual_cue_classification_node = human_robot_interaction.visual_cue_classification_node:main',
            'visual_descriptor_processing_node = human_robot_interaction.visual_descriptor_processing_node:main',
            
            'correlator_node = human_robot_interaction.correlator_node:main',
            'fusion_node = human_robot_interaction.fusion_node:main',
            'plotting_node = human_robot_interaction.plotting_node:main',
        ],
    },
)