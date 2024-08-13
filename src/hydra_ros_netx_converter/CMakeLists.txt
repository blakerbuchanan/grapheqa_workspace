cmake_minimum_required(VERSION 3.0.2)
project(hydra_ros_netx_converter)

find_package(spark_dsg REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  hydra_msgs
  message_generation
  spark_dsg
)


## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  hydra_msgs
  spark_dsg
)

catkin_python_setup()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/hydra_ros_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
