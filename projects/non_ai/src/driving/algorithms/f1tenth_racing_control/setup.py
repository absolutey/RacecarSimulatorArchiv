from glob import glob
from setuptools import find_packages, setup


package_name = "f1tenth_racing_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neo",
    maintainer_email="neo@example.com",
    description="Deterministic model-aided pursuit fallback",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "r_b0_controller = f1tenth_racing_control.node:main",
            "actuator_guard = f1tenth_racing_control.actuator_guard_node:main",
            "independent_guard_watchdog = f1tenth_racing_control.independent_watchdog_node:main",
        ]
    },
)
