from setuptools import find_packages
from setuptools import setup

setup(
    name='r2d2',
    version='1.0.0',
    packages=find_packages(
        include=('r2d2', 'r2d2.*')),
)
