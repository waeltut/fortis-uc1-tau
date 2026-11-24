from glob import glob

from setuptools import find_packages, setup

package_name = "fortis_tk24_tts"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/tts.yaml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fortis_dev",
    maintainer_email="luka.boljevic@xlab.si",
    description="ROS 2 wrapper around the text_to_speech (Piper) core.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "tts_node = fortis_tk24_tts.node:main",
        ],
    },
)
