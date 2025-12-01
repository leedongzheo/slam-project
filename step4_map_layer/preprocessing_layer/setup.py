from setuptools import setup
import os
from glob import glob

package_name = 'preprocessing_layer'

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
    maintainer='Preprocessing Layer Maintainer',
    maintainer_email='maintainer@example.com',
    description='Depth preprocessing pipeline converting images to filtered point clouds and LaserScan.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'preprocessing_layer_node = preprocessing_layer.preprocessing_layer_node:main',
        ],
    },
)
