"""
NRF24L01+
---------

Nordic nrf24l01+ low power 2.4GHz radio driver for python.


"""

# Setuptools
from setuptools import setup, find_packages
import sys


if sys.version_info < (3, 5):
    raise Exception("Modis Lock requires Python 3.5 or higher.")


with open('./README.md', encoding='utf-8') as f:
    readme = f.read()

requires = [i.strip() for i in open("./requirements.txt").readlines()]


setup(
    name='nrf24l01',

    # Versions should comply with PEP440.  For a discussion on single-sourcing
    # the version across setup.py and the project code, see
    # https://packaging.python.org/en/latest/single_source_version.html
    version='0.0.1',

    description='NRF24L01 library for the Nordic low power 2.4GHz radio for the Raspberry Pi',
    long_description=readme,

    # The project's main homepage.
    url='https://github.com/Modis-GmbH/ModisLock-WebAdmin',

    # Choose your license
    license='GPL',

    # Author details
    author='Richard Lowe',
    author_email='richard@modis.io',

    # What does your project relate to?
    keywords=['nrf24l01', 'raspberry pi', 'wireless', 'low power radio'],

    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        # How mature is this project? Common values are
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        "Development Status :: 4 - Beta",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.5",
        "Environment :: Console",
        "Programming Language :: Python",
        "Intended Audience :: Developers",
        "License :: OSI Approved",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "Operating System :: POSIX :: Linux",
        "Topic :: Security"
    ],

    platforms=['Linux', 'Raspberry Pi'],

    zip_safe=False,

    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    packages=find_packages(exclude=['docs', 'tests*']),

    # List run-time dependencies here.  These will be installed by pip when
    # your project is installed. For an analysis of "install_requires" vs pip's
    # requirements files see:
    # https://packaging.python.org/en/latest/requirements.html
    install_requires=requires,

    # If your project only runs on certain Python versions, setting the python_requires argument to the appropriate
    # PEP 440 version specifier string will prevent pip from installing the project on other Python versions.
    python_requires='>=3.5',

    # If there are data files included in your packages that need to be
    # installed, specify them here.  If using Python 2.6 or less, then these
    # have to be included in MANIFEST.in as well.
    include_package_data=True
)