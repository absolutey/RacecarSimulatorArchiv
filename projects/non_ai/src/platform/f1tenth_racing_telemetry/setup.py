from setuptools import find_packages, setup

package_name = "f1tenth_racing_telemetry"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="neo",
    maintainer_email="neo@example.com",
    description="Passive F1TENTH racing observability",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "flight_recorder = f1tenth_racing_telemetry.recorder:main",
        "analyze_run = f1tenth_racing_telemetry.analyzer:main",
        "spectator_hud = f1tenth_racing_telemetry.hud:main",
    ]},
)
