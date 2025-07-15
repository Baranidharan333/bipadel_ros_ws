from setuptools import find_packages, setup
import os
package_name = 'custom_ultrasonic_laserscanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', ['launch/fake_scan.launch.py']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baranidharan',
    maintainer_email='baranidharan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_laserscanner_node = custom_ultrasonic_laserscanner.fake_laserscanner_node:main',
        ],
    },
)
