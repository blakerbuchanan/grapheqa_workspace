"""Catkin setup for vlm_planner_semnav."""
from setuptools import setup

setup(
    name='vlm_planner_semnav',
    version='0.0.1',
    packages=['vlm_planner_semnav'],
    package_dir={'': 'src'},
    install_requires=[
        'spark_dsg',  # Include spark_dsg as a dependency
    ],
    scripts=[
        'scripts/vlm_planner_eqa_ros.py',  # Include your ROS node scripts
    ],
    author='Blake Buchanan',
    author_email='blakerbuchanan@gmail.com',
    description='My package that depends on spark_dsg',
    license='BSD',
    tests_require=['pytest'],
)
