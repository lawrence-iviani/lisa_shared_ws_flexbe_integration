#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['lisa_shared_ws_flexbe_integration_flexbe_states'],
    package_dir = {'': 'src'}
)

setup(**d)
