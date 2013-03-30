#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_obj = generate_distutils_setup(
                  package=['visual_servo'],
                  package_dir={'visual_servo': 'src'}
                )

setup(**setup_obj)
