#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_obj = generate_distutils_setup(
                  packages=['visual_servo'],
                  package_dir={'': 'src'}
                )

setup(**setup_obj)
