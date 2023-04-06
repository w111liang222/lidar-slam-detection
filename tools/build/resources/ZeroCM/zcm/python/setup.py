from setuptools import setup, find_packages
from setuptools.extension import Extension
from Cython.Build import cythonize
import os

from ctypes.util import find_library
if find_library("zcm") is None:
    print("\n\n"
          "##################################################################################################\n"
          "#  Warning! You need to install the ZeroCM library before installing the zerocm python package!  #\n"
          "#  Build it from source here: https://github.com/ZeroCM/zcm                                      #\n"
          "##################################################################################################\n\n")

setup(
    name='zerocm',
    version='1.1.5',
    url='https://github.com/ZeroCM/zcm',
    license='LGPL',
    packages=find_packages(),
    include_dirs = ['.'],
    ext_modules=cythonize([
        Extension(
            "zerocm",
            ["zcm/python/zerocm.pyx"],
            libraries=['zcm']
        )
    ]),
)