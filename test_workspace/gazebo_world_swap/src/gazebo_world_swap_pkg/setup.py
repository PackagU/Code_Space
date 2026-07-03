from glob import glob
from setuptools import setup

package_name = "gazebo_world_swap_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lee",
    maintainer_email="JunhyungLee25@users.noreply.github.com",
    description="Simulation-only Gazebo world swap node for elevator map-switch PoC",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "world_swap_node = gazebo_world_swap_pkg.world_swap_node:main",
        ],
    },
)
