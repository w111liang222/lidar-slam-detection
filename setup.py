import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

from hardware.platform_common import BOARD_NAME, MACHINE

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-Wno-dev', '-DCMAKE_BUILD_TYPE=' + cfg, '-DBOARD_NAME=' + BOARD_NAME]

        if BOARD_NAME in ["AGX-Orin-32GB", "NX-Orin"]:
            build_thread = 8
        else:
            build_thread = 4

        os.system(''' echo "Board {}, use {} threads for building" '''.format(BOARD_NAME, build_thread))
        build_args += ['--', '-j'+str(build_thread)]

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)


setup(
    name='perception',
    version='1.0.0',
    author='LiangWang',
    author_email='15lwang@alumni.tongji.edu.cn',
    description='LiDAR SLAM & Detection',
    long_description='',
    ext_modules=[CMakeExtension('perception')],
    data_files=[('lib/', [f'sensor_driver/inference/libspconv/lib/{MACHINE}/libspconv.so'])],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
