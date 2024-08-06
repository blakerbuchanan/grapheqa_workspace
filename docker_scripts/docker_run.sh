#!/bin/bash

# Name of the Docker container
CONTAINER_NAME="ros-noetic-for-hydra"

# Docker image to use
DOCKER_IMAGE="blakerbuchanan/ros-noetic-for-hydra:0.0.1"

# Path to the workspace directory
WORKSPACE_DIR="$(pwd)"

# Environment variables
DISPLAY_VAR=$DISPLAY
SSH_AUTH_SOCK_VAR=$SSH_AUTH_SOCK

# Run the Docker container with the appropriate arguments
docker run -it \
  --name $CONTAINER_NAME \
  --privileged \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY_VAR \
  -e QT_X11_NO_MITSHM=1 \
  -v $SSH_AUTH_SOCK_VAR:/run/ssh-agent \
  -e SSH_AUTH_SOCK=/run/ssh-agent \
  -v $WORKSPACE_DIR:/workspace:cached \
  --mount type=volume,src=ros-noetic-for-hydra,dst=/opt/ros_noetic_for_hydra_ws,consistency=cached \
  --user guest \
  $DOCKER_IMAGE \
  /bin/bash
