import os

from setuptools import setup

package_name = "map_layer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "config"),
            ["config/gmapping_params.yaml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            ["launch/map_layer.launch.py"],
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="TurtleBot3 GMapping",
    maintainer_email="maintainer@example.com",
    description="Pure-Python GMapping map layer for TurtleBot3 simulation (ROS 2 Jazzy)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gmapping_node = map_layer.gmapping_node:main",
        ],
    },
)
