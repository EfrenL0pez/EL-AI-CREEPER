from setuptools import setup
import os
from glob import glob

package_name = 'charged_creeper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={package_name: 'nodes'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Efren Lopez',
    maintainer_email='efren@example.com',
    description='El AI Creeper robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_node = charged_creeper.keyboard_node:main',
            'communication_node = charged_creeper.communication_node:main',
        ],
    },
)