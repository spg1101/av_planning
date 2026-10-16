from setuptools import find_packages, setup

package_name = "planning_sim"

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
    description="Stand-ins for other subteams (adapter, sim, controller, perception)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gym_adapter = planning_sim.gym_adapter:main",
            "kinematic_sim = planning_sim.kinematic_sim:main",
            "pure_pursuit_stub = planning_sim.pure_pursuit_stub:main",
            "fake_perception = planning_sim.fake_perception:main",
            "canned_trajectory = planning_sim.canned_trajectory_pub:main",
            "waypoint_recorder = planning_sim.waypoint_recorder:main",
        ],
    },
)
