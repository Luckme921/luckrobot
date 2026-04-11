from setuptools import find_packages
from setuptools import setup

setup(
    name='fast_lio_map',
    version='0.0.0',
    packages=find_packages(
        include=('fast_lio_map', 'fast_lio_map.*')),
)
