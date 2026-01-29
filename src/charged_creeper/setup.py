from setuptools import find_packages, setup

package_name = 'charged_creeper'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6SPEED',
    maintainer_email='6speed@todo.com',
    description='El AI Creeper Robot Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication_node = nodes.communication_node:main',
            'keyboard_node = nodes.keyboard_node:main',
        ],
    },
)
