from setuptools import setup
import os
from glob import glob

package_name = 'charged_creeper'

setup(
    name=package_name,
    version='1.0.0',
    packages=['nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 package for controlling a robot with keyboard input',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_node = nodes.keyboard_node:main',
            'communication_node = nodes.communication_node:main',
        ],
    },
)
