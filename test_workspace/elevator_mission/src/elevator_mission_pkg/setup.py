from glob import glob
from setuptools import setup

package_name = "elevator_mission_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("../../config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="PackagU",
    maintainer_email="inonewater@users.noreply.github.com",
    description="Delivery mission behavior tree for isolated elevator PoC",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "delivery_mission_node = elevator_mission_pkg.delivery_mission_node:main",
        ],
    },
)
