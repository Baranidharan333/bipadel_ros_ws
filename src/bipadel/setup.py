from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bipadel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*')),
        (os.path.join('share',package_name,'meshes'),glob('meshes/*')),
        ('share/' + package_name + '/config', ['config/rviz_config.rviz']),
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
        'random_joint_publisher = bipadel.random_joint_publisher:main',
        'imu_node = bipadel.imu_node:main',
        'SLAM = bipadel.SLAM:main',
        'imu_broadcaster = bipadel.imu_broadcaster:main',
        'UDP_reciver = bipadel.UDP_reciver:main',
        
        ],
    },
)
