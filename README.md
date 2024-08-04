# Hydra for ROS1
This repo uses Docker to set up and run the implementation at this link: https://github.com/MIT-SPARK/Hydra/tree/main?tab=readme-ov-file

This repo is not claiming to have developed anything there, but is soley for setting things up in Docker since the implementation relies on ROS Noetic and Ubuntu 20.04.

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
Source the ROS setup. First navigate to the workspace directory.
```bash
cd /workspaces/ros-noetic-for-hydra
```
Then source noetic:
```bash
source /opt/ros/noetic/setup.bash
```
Initalize your catkin workspace and set build configs:
```bash
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
```
You can now clone hydra, initialize the submodules, and install ros dependencies:
```bash
cd src
git clone git@github.com:MIT-SPARK/Hydra.git hydra
vcs import . < hydra/install/hydra.rosinstall
rosdep install --from-paths . --ignore-src -r -y
```
Finally, build:
```bash
cd ..
catkin build
```
