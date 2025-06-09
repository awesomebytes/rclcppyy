from setuptools import setup
import os
from glob import glob

package_name = 'rclcppyy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam Pfeiffer',
    maintainer_email='sammypfeiffer@gmail.com',
    description='rclcpp bindings via cppyy and examples on how to use cppyy in ROS2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Entry points are not needed here as we're using the CMake install method
        ],
    },
) 