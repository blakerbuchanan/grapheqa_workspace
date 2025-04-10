#!/bin/bash
#
# Script to build the docker image
# -u        Upload the build image to hub.docker.com

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source "${DIR}/docker_tag.sh"

_do_upload=false
while getopts "u" o; do
    case "${o}" in
        u) _do_upload=true
            ;;
        *)
            exit 1;
            ;;
    esac
done

docker_registry_image=blakerbuchanan/grapheqa_for_stretch

docker build \
       --build-arg TAG=${TAG} \
       --platform linux/amd64 \
       -t ${docker_registry_image}:${TAG} -f Dockerfile .

# Explicitly docker logout and docker login to make sure the correct docker repo will be pushed to if upload is needed
