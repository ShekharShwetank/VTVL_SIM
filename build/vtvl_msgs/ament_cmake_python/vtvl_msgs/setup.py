from setuptools import find_packages
from setuptools import setup

setup(
    name='vtvl_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('vtvl_msgs', 'vtvl_msgs.*')),
)
