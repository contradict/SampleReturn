#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
          packages=["samplereturn"],
          package_dir={'' : 'src'},          
          scripts=[
               'nodes/camlogger.py',
               'nodes/image_desync.py',
               'nodes/lights.py',
               'nodes/announce_voltage.py',
               'nodes/pause_switch.py',
               ],
          )

setup(**d)
