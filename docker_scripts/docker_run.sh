#!/bin/bash

# Name of the Docker container
CONTAINER_NAME="ros-noetic-for-hydra"

# Docker image to use
DOCKER_IMAGE="blakerbuchanan/ros-noetic-for-hydra-with-bindings:0.0.1"

# Path to the workspace directory
WORKSPACE_DIR="$(pwd)"

# Environment variables
SSH_AUTH_SOCK_VAR=$SSH_AUTH_SOCK

# Run the Docker container with the appropriate arguments
docker run -it \
  --name $CONTAINER_NAME \
  --privileged \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="XAUTHORITY:$XAUTHORITY" \
  --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e LD_LIBRARY_PATH=/usr/lib/nvidia-535:$LD_LIBRARY_PATH \
  -v $SSH_AUTH_SOCK_VAR:/run/ssh-agent \
  -e SSH_AUTH_SOCK=/run/ssh-agent \
  -v $WORKSPACE_DIR:/workspace:cached \
   --runtime nvidia \
  --user guest \
  $DOCKER_IMAGE \
  /bin/bash
