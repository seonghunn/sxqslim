import os
import re
import subprocess
import sys
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DPYTHON_EXECUTABLE=' + sys.executable,
            '-DCMAKE_BUILD_TYPE=Release',  # 릴리즈 모드로 빌드
            '-DCMAKE_CXX_FLAGS=-O3'  # O3 최적화 활성화
            # 필요한 경우 여기에 추가 CMake 옵션을 포함시킵니다.
        ]
        cfg = 'Release'  # 릴리즈 모드를 명시적으로 지정
        build_args = ['--config', cfg]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(['cmake', '--build', '.', '--target', ext.name] + build_args, cwd=self.build_temp)

setup(
    name='sxqslim',
    version='1.0',
    author='seonghunn',
    author_email='your.email@example.com',
    description='self intersection free qslim-based mesh decimation',
    long_description='',
    packages=find_packages(),
    ext_modules=[CMakeExtension('sxqslim', sourcedir='.')],
    cmdclass=dict(build_ext=CMakeBuild),
)
