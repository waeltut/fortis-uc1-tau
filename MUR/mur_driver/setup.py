from setuptools import find_packages, setup
from glob import glob
import os

package_name = "mur_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools", 'trimesh>=3.9,<5'],
    zip_safe=True,
    maintainer="fortis_dev",
    maintainer_email="todo@todo.com",
    description="Bringup package for the MiR dual-UR5e mobile manipulator.",
    license="TODO",
    entry_points={
        "console_scripts": [
            'mur_footprint = mur_driver.mur_footprint:main',
        ],
    },
)