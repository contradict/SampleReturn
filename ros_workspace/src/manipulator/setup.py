#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
           scripts=['src/manipulator_node.py',
                    'src/manipulator_test_node.py'],
           packages=['manipulator'],
           package_dir={'manipulator': 'src'}
          )

setup(**d)
