from setuptools import find_packages, setup

package_name = "planning_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Planning Lead",
    maintainer_email="planning-lead@example.com",
    description="ROS 2 wrappers for the global, behavior and local planners",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_planner = planning_nodes.global_planner_node:main",
            "behavior_planner = planning_nodes.behavior_node:main",
            "local_planner = planning_nodes.local_planner_node:main",
        ],
    },
)
