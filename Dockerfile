ARG TAG=0.0.1
ARG DEBIAN_FRONTEND=noninteractive

# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-core-focal

###################################################################################################
# Set up guest user  and tools for development
ARG DEV_GROUP_ID=8888
ARG ID_NAME=guest
ENV ID_NAME=${ID_NAME}
# Create default users
RUN groupadd -g ${DEV_GROUP_ID} developers \
    && useradd -ms /bin/bash $ID_NAME \
    && usermod -aG sudo $ID_NAME \
    && usermod -aG developers $ID_NAME \
    && echo "${ID_NAME}:${ID_NAME}" | chpasswd
USER $ID_NAME
WORKDIR /home/${ID_NAME}
RUN mkdir /home/${ID_NAME}/.ssh

###################################################################################################
# Build the image as root from the / folder
USER root
WORKDIR /

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-vcstool \
    python3-catkin-tools \
    git \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install behaviortreecpp dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    libzmq3-dev
