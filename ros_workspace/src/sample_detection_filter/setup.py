#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_obj = generate_distutils_setup(
                  package=['sample_detection_filter'],
                  package_dir={'sample_detection_filter': 'src'}
                )

setup(**setup_obj)
