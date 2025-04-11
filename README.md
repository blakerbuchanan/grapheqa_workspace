# GraphEQA Project Workspace Configuration
This repo provides tools that use Docker to set up and run the implementation at this link: https://github.com/blakerbuchanan/Hydra . This is a fork of the implementation at https://github.com/MIT-SPARK/Hydra, with modifications that support the "GraphEQA" project.

Owners and collaborators of this repo are not claiming to have developed anything original to Hydra, but are (at the time of this commit, anyway) responsible for developing everything needed to use the implementation in Docker. Containerizing this seemed useful since the developers state that it is tested on ROS Noetic and Ubuntu 20.04.

## Docker setup
### Prerequisites
Install docker.

### Configure development environment
#### Set up the Docker image locally
Run the following script to build the docker image locally.

```bash
./docker_scripts/docker_build.sh
```

The following script will run a container 

```bash
./docker_scripts/docker_run.sh
```

If the docker container is already running and you want to execute the container in another terminal instance, do:

``` bash
docker exec -it grapheqa-for-stretch bash
```

##### Build and run the example
TODO
