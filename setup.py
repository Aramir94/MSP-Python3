import os
from setuptools import setup, find_packages


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="msp-python3",
    version="0.2.7",

    author="Luke A. Rohl",
    author_email="Luke.A.Rohl@gmail.com",
    url="https://github.com/LukeARohl/MSP-Python3",

    description="MultiWii Serial Protocol (MSP) API for python3",
    long_description=read('README.md'),
    long_description_content_type="text/markdown",

    license="MIT",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 2 - Pre-Alpha",
    ],

    packages=find_packages(include=['msp', 'msp.*']),
    install_requires='pyserial'
)
