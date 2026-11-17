from setuptools import setup, find_packages
package_name='deep_racing_next'
setup(name=package_name,version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
             ('share/'+package_name,['package.xml'])],
 install_requires=['setuptools','numpy','PyYAML'],zip_safe=True,
 maintainer='Deep Racing Project',maintainer_email='noreply@example.invalid',
 description='Reorganized Deep Racing contracts (non-actuating)',license='Apache-2.0')
