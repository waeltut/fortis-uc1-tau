from glob import glob

from setuptools import find_packages, setup

package_name = "fortis_tk22_object_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/assets/beko", glob("assets/beko/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fortis_dev",
    maintainer_email="luka.boljevic@xlab.si",
    description="FORTIS TK2.2 obstacle detection (YOLOE + depth) for the BEKO pilot.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "obstacle_detection = "
            "fortis_tk22_object_detection.obstacle_detection_node:main",
        ],
    },
)
