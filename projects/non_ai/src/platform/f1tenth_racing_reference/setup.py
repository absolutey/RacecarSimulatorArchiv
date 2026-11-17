from glob import glob
from setuptools import find_packages, setup


package_name = "f1tenth_racing_reference"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neo",
    maintainer_email="neo@example.com",
    description="Shared nominal active trajectory reference",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nominal_active_reference = f1tenth_racing_reference.node:main",
        ]
    },
)
