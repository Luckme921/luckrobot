from setuptools import find_packages
from setuptools import setup

setup(
    name='livox_ros_driver2',
    version='1.0.0',
    packages=find_packages(
        include=('livox_ros_driver2', 'livox_ros_driver2.*')),
)
