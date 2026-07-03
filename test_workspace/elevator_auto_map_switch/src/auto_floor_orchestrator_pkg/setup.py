from setuptools import setup

package_name = "auto_floor_orchestrator_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/auto_floor_orchestrator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lee",
    maintainer_email="JunhyungLee25@users.noreply.github.com",
    description="Automatic floor/map switch orchestrator PoC",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "auto_floor_orchestrator_node = "
            "auto_floor_orchestrator_pkg.auto_floor_orchestrator_node:main",
        ],
    },
)
