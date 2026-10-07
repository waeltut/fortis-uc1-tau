from glob import glob
from setuptools import find_packages, setup

setup(
    name='mur_base_selector', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mur_base_selector']),
        ('share/mur_base_selector', ['package.xml', 'README.md']),
        ('share/mur_base_selector/launch', glob('launch/*.launch.py')),
        ('share/mur_base_selector/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='MUR maintainers', maintainer_email='maintainers@example.com',
    description='Service-driven reachability candidate selection with Nav2 footprint and path validation',
    license='Apache-2.0',
    entry_points={'console_scripts': ['base_selector = mur_base_selector.node:main']},
)
