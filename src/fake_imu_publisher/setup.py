from setuptools import find_packages, setup

package_name = 'fake_imu_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
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
            'fake_imu_node = fake_imu_publisher.fake_imu_node:main',
            'imu_tf_broadcaster = fake_imu_publisher.imu_tf_broadcaster:main',        ],
    },
)
