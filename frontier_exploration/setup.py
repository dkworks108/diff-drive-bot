import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'frontier_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='bumble',
    maintainer_email='dkworks108@gmail.com',
    description='Autonomous frontier-based exploration package for ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = frontier_exploration.explorer_node:main',
        ],
    },
)

