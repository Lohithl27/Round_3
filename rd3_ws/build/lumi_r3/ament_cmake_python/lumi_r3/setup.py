from setuptools import find_packages
from setuptools import setup

setup(
    name='lumi_r3',
    version='1.0.0',
    packages=find_packages(
        include=('lumi_r3', 'lumi_r3.*')),
)
