from setuptools import find_packages, setup

package_name = "planning_world_model"

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
    description="World model: ego state, costmap, tracked and predicted obstacles",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "world_model = planning_world_model.world_model_node:main",
            "tracker = planning_world_model.tracker_node:main",
        ],
    },
)
