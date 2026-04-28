from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mir_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/mir_description']),
    ('share/mir_description', ['package.xml']),

    # launch
    (os.path.join('share', 'mir_description', 'launch'),
        glob('launch/*.py')),

    # urdf main files
    (os.path.join('share', 'mir_description', 'urdf'),
        glob('urdf/*.xacro')),

    # urdf include files (THIS FIXES YOUR ERROR)
    (os.path.join('share', 'mir_description', 'urdf/include'),
        glob('urdf/include/*.xacro')),

    # meshes (visual + collision)
    (os.path.join('share', 'mir_description', 'meshes/visual'),
        glob('meshes/visual/*')),
    (os.path.join('share', 'mir_description', 'meshes/collision'),
        glob('meshes/collision/*')),
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
        ],
    },
)
