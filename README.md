# Hydra for ROS1

### Prerequisites
Install docker.

### Configure development environment
We first want to configure a development environment in which we can build the docker image. This will also create a docker volume that will mount to the workspace. To create the environment profile, do

```bash
./docker_scripts/create_environment_profile.sh
```

We now want to build the docker image. We could probably pull the docker image down from my public repository on docker hub, but I haven't tested that so just build it locally for now.

```bash
./docker_scripts/docker_build.sh
```

You should now be able to run the docker image using

```bash
./docker_scripts/docker_run.sh
```



### Build and run the example