from setuptools import find_packages
from setuptools import setup

setup(
    name='orbbec_camera_msgs',
    version='2.7.6',
    packages=find_packages(
        include=('orbbec_camera_msgs', 'orbbec_camera_msgs.*')),
)
