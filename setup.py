from setuptools import setup
import os
from glob import glob

package_name = 'mec_wheel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sylee',
    maintainer_email='user@todo.todo',
    description='Mecanum wheel control package for Aruco tracking',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mec_wheel_node = mec_wheel.mec_wheel:main'
        ],
    },
)