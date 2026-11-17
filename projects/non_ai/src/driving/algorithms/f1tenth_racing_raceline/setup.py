from glob import glob
from setuptools import find_packages, setup


package_name = "f1tenth_racing_raceline"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/data", glob("data/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neo",
    maintainer_email="neo@example.com",
    description="Deterministic offline baseline racing-line generation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generate_baseline_racing_line = f1tenth_racing_raceline.cli:main",
            "generate_baseline_speed_profile = f1tenth_racing_raceline.speed_cli:main",
            "build_racing_trajectory = f1tenth_racing_raceline.trajectory_cli:main",
            "generate_scp_racing_trajectory = f1tenth_racing_raceline.scp_cli:main",
        ]
    },
)
