from setuptools import setup
from os import environ
from setuptools import find_packages

package_name = environ['ROS2PKG']
nodes = environ['ROS2NODES'].split(" ")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FabricateIO',
    maintainer_email='contact@fabricate.io',
    description='See package.xml',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '%s = %s.%s:main' % (node, package_name, node)
            for node in nodes
        ],
    },
)
