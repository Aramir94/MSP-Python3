import setuptools

setuptools.setup(
    name="msp-python3",
    version="1.0",
    python_requires='>=3.7',

    author="Luke A. Rohl",
    author_email="Luke.A.Rohl@gmail.com",
    url="https://github.com/LukeARohl/MSP-Python3",

    description="MultiWii Serial Protocol (MSP) API for python3",

    license="MIT",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],

    packages=setuptools.find_packages(),
)
