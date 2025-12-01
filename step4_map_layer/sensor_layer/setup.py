from setuptools import setup
import os
from glob import glob

package_name = 'sensor_layer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sensor Layer Maintainer',
    maintainer_email='maintainer@example.com',
    description='Sensor aggregation and republishing layer for TurtleBot3 Gazebo simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_layer_node = sensor_layer.sensor_layer_node:main',
        ],
    },
)
