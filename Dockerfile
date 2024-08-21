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
    python3-pip \
    libeigen3-dev \
    libzmqpp-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    ros-noetic-tf2-eigen \
    ros-noetic-interactive-markers \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-image-proc \
    ros-noetic-depth-image-proc \
    ros-noetic-rviz \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    ros-noetic-gtsam \
    libopencv-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install networkx

# The idea behind the following code is to build Hydra as part of the Docker image
# This doesn't currently work...
# RUN mkdir /home/${ID_NAME}/catkin_ws

# RUN mkdir /home/${ID_NAME}/catkin_ws/src

# RUN cd /home/${ID_NAME}/catkin_ws

# # Source the ROS setup script and ensure it's applied in the same shell session
# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# RUN catkin init && \
#     catkin config -DCMAKE_BUILD_TYPE=Release

# RUN cd /home/${ID_NAME}/catkin_ws/src

# # Create a known_hosts file
# RUN mkdir -p /root/.ssh
# RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# # Add the SSH key to the Docker image (use build argument for the SSH key)
# ARG SSH_PRIVATE_KEY
# RUN echo "$SSH_PRIVATE_KEY" > /root/.ssh/id_ed25519 && chmod 600 /root/.ssh/id_ed25519

# RUN git clone git@github.com:MIT-SPARK/Hydra.git hydra

# RUN vcs import . < hydra/install/hydra.rosinstall

# ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
# RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
# RUN echo 'APT::Get::Assume-Yes "true";' > /etc/apt/apt.conf.d/90force-yes

# USER $ID_NAME

# RUN rosdep init && \
#     rosdep update --rosdistro $ROS_DISTRO && \
#     rosdep install --rosdistro $ROS_DISTRO --from-paths . --ignore-src -r -y --verbose

# We need to do this to install the Python bindings for spark_dsg
# cd src/spark_dsg
# pip install -e .
# pip install networkx

# RUN cd /home/${ID_NAME}/catkin_ws
