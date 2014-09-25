#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_obj = generate_distutils_setup(
                  packages=['motion_planning'],
                  package_dir={'': 'src'},
                  scripts=[
                      'nodes/simple_driving.py'
                      ]
                )

setup(**setup_obj)
