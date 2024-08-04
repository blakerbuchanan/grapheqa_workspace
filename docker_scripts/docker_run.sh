#!/bin/bash
#
# Runs the docker container

ws_host_dir=$(realpath ${DIR}/../../)

IMAGE="blakerbuchanan/ros-noetic-for-hydra"
TAG=0.0.1
DETACHED=""

CONTAINER_CMD='/bin/bash'
docker run \
    --privileged \
    --name ros-noetic-for-hydra \
    --hostname "$(hostname)-ros-noetic-for-hydra" \
    --restart always \
    $DETACHED \
    -it \
    --network host \
    --platform linux/amd64 \
    -v "${ws_host_dir}":/workspaces:rw \
    "${IMAGE}:${TAG}" "${CONTAINER_CMD}"