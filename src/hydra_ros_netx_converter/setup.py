"""Catkin setup for hydra_ros_netx_converter."""
from setuptools import setup

setup(
    name='hydra_ros_netx_converter',
    version='0.0.1',
    packages=['hydra_ros_netx_converter'],
    package_dir={'': 'src'},
    install_requires=[
        'spark_dsg',  # Include spark_dsg as a dependency
    ],
    scripts=[
        'scripts/hydra_ros_listener.py',  # Include your ROS node scripts
    ],
    author='Blake Buchanan',
    author_email='blakerbuchanan@gmail.com',
    description='My package that depends on spark_dsg',
    license='BSD',
    tests_require=['pytest'],
)
