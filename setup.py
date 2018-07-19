#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    version='0.1.0',
    packages=['imitation_learning'],
    package_dir={'': 'src/imitation_learning'},
    scripts=['tests/imports.py']
)

setup(**d)
