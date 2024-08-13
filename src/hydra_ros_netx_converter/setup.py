"""Catkin setup for hydra_ros_netx_converter."""
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(packages=["hydra_ros_netx_converter"], package_dir={"": "src"})
setup(**setup_args)
