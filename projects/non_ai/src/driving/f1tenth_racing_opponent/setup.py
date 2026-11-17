from setuptools import find_packages, setup
package_name = "f1tenth_racing_opponent"
setup(name=package_name, version="0.1.0", packages=find_packages(), data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]), ("share/" + package_name, ["package.xml"])], install_requires=["setuptools"], zip_safe=True, maintainer="neo", maintainer_email="neo@example.com", description="Fixed Red baseline", license="Apache-2.0", entry_points={"console_scripts": ["red_baseline_controller = f1tenth_racing_opponent.controller:main"]})
