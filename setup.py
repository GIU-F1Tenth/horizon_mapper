from setuptools import find_packages, setup
from glob import glob

package_name = 'horizon_mapper'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name, glob('horizon_mapper/*.csv')),
]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammedazab',
    maintainer_email='mohammedazab@example.com',
    description='Horizon Mapper - Trajectory processing and publishing package for F1TENTH MPC controller',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'horizon_mapper_node = horizon_mapper.horizon_mapper_node:main',
        ],
    },
)
