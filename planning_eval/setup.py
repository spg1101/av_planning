from setuptools import find_packages, setup

package_name = "planning_eval"

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
    description="Evaluation: localization error vs ground truth, metrics, regression tests",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_error = planning_eval.localization_error:main",
        ],
    },
)
