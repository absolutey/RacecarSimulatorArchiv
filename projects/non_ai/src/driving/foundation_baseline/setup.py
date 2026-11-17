from setuptools import find_packages, setup
package_name = "foundation_baseline"
setup(name=package_name, version="0.1.0", packages=find_packages(), data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]), ("share/" + package_name, ["package.xml"])], install_requires=["setuptools"], zip_safe=True, maintainer="neo", maintainer_email="neo@example.com", description="Qualification-only Blue baseline", license="Apache-2.0", entry_points={"console_scripts": ["blue_foundation_controller = foundation_baseline.controller:main"]})
