import os
from glob import glob

from setuptools import find_packages, setup

package_name = "planning_carla"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Planning Lead",
    maintainer_email="planning-lead@example.com",
    description="CARLA platform adapter and map export tools",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carla_adapter = planning_carla.carla_adapter:main",
            "export_lane_graph = planning_carla.tools.export_lane_graph:main",
            "export_vehicle_params = planning_carla.tools.export_vehicle_params:main",
            "list_spawn_points = planning_carla.tools.list_spawn_points:main",
        ],
    },
)
