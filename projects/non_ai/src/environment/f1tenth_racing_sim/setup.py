from glob import glob
from setuptools import find_packages, setup

package_name = "f1tenth_racing_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    package_data={"f1tenth_racing_sim.safety": ["native_stop_proof.cpp"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps/iccas2025", glob("maps/iccas2025/*")),
        ("share/" + package_name + "/maps/spielberg", glob("maps/spielberg/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neo",
    maintainer_email="neo@example.com",
    description="Clean competition environment simulator",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["simulator = f1tenth_racing_sim.simulator:main"]},
)
