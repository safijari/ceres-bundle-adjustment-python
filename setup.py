# setup.py

from setuptools import setup
from Cython.Build import cythonize

setup(
    name='bundle_adjustment',
    ext_modules=cythonize("bundle_adjustment.pyx"),
)