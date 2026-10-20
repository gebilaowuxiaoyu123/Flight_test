import os
from glob import glob

from setuptools import setup

package_name = 'flight_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiki_wu',
    maintainer_email='wiki_wu@localhost',
    description='Browser-based RC transmitter for PX4 SITL',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ui_server = flight_ui.server:main',
        ],
    },
)
