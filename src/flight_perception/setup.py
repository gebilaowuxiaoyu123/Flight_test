import os
from glob import glob

from setuptools import setup

package_name = 'flight_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiki_wu',
    maintainer_email='wiki_wu@localhost',
    description='Optical flow logging and depth camera helpers',
    license='MIT',
    entry_points={
        'console_scripts': [
            'optical_flow_logger = flight_perception.optical_flow_logger:main',
            'min_depth = flight_perception.min_depth:main',
        ],
    },
)
