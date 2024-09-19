# SemNav Project Workspace Configuration
This repo does a few things:

1) Provides instructions for how to set up a workspace on Ubuntu 20.04 to be able to run everything associated with our semantic navigation work.

2) It provides tools that use Docker to set up and run the implementation at this link: https://github.com/blakerbuchanan/Hydra . This is a fork of the implementation at https://github.com/MIT-SPARK/Hydra, with modifications that support the "SemNav" project.

Owners and collaborators of this repo are not claiming to have developed anything original to Hydra, but are (at the time of this commit, anyway) responsible for developing everything needed to use the implementation in Docker. Containerizing this seemed useful since the developers state that it is tested on ROS Noetic and Ubuntu 20.04.

3) Implements a basic custom listener node that listens for Hydra to publish a dynamic scene graph, of DynamicSceneGraph type defined within spark_dsg, and converts its contents into a JSON file using tools native to spark_dsg and networkx. The message published contains a serialized representation of the scene graph, requiring us to deserialize it upon receipt by the 'HydraSceneGraphListenerNode'. Since even the serialized scene graph contains data types custom to spark_dsg's DynamicSceneGraph type, we also need to filter out data types that are not JSON serializable through networkx's `node_link_data` function.

## Setting up your workspace on Ubuntu 20.04
This set of instructions is only for local Ubuntu 20.04 installations. Please refer to the Docker setup section of this README if you are on any other Ubuntu OS version.

0) If you don't have ROS Noetic, install it: https://wiki.ros.org/ROS/Installation

1) Then do:

``` bash
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

Set up rosdep:

``` bash
sudo rosdep init
rosdep update
```

Clone this repo:

``` bash
git clone git@github.com:blakerbuchanan/semnav_workspace.git
```

`cd semnav_workspace`

``` bash
source /opt/ros/noetic/setup.bash
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release

cd src
git clone git@github.com:blakerbuchanan/Hydra.git
vcs import . < hydra/install/hydra.rosinstall
rosdep install --from-paths . --ignore-src -r -y

cd ..
catkin build
```

At this point we can make sure Hydra is installed correctly by doing the following.

``` bash
source devel/setup.bash
roslaunch hydra_ros uhumans2.launch
```

An RViz window should open. If nothing crashes, you are probably good.

To test further, download the uhumans2 dataset at https://drive.usercontent.google.com/download?id=1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb&authuser=0 .

Then do:

``` bash
rosbag play path/to/rosbag --clock
```

You should see the scene graph, mesh, etc., begin populating in the RViz window that opened.

2) Install the Hydra Python bindings

First create a conda environment:

`conda create -n "hydra_semnav" python=3.8`

``` bash
conda activate hydra_semnav
```

Now we will install editable versions of the Spark-DSG and Hydra Python bindings:

``` bash
# required to expose DSG python bindings
pip install -e "src/spark_dsg[viz]"
pip install -r python/build_requirements.txt
pip install -e .
```

3) Install Habitat via conda

`conda install habitat-sim -c conda-forge -c aihabitat`

4) We need a few other things:

``` bash
pip install rerun-sdk
pip install opencv
pip install openai
```

The OpenAI API requires an API key. Add the following to your .bashrc:

`export OPENAI_API_KEY=<YOUR_OPENAI_KEY>`


## Docker setup
### Prerequisites
Install docker.

### Configure development environment
We first want to configure a development environment in which we can build the docker image. This will also create a docker volume that will mount to the workspace. After cloning this repo, navigate to `ros_hydra_in_docker`. To create the environment profile, do

```bash
./docker_scripts/create_environment_profile.sh
```

We now want to build the docker image. Before executing the following script, change the docker build argument within the script to correspond to the SSH key local to your system. This provides access such that submodules clone and initialize without issue. The build argument in question looks like `build-arg SSH_PRIVATE_KEY`. 

```bash
./docker_scripts/docker_build.sh
```

Navigate to the `ros_hydra_in_docker` directory. You should now be able to run the docker image using

```bash
./docker_scripts/docker_run.sh
```

If the docker container is already running and you want to execute the container in another terminal instance, do:

``` bash
docker exec -it ros-noetic-for-hydra bash
```


### Build and run the example
Within the same terminal instance in which the docker container is running, source the ROS setup. First navigate to the workspace directory in the docker container.
```bash
cd /workspace
```
This workspace should contain the contents of the `ros_hydra_in_docker` directory.

Source noetic:
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
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

Next, we will need to install the python bindings for spark_dsg so we can use tools for handling objects of the DynamicSceneGraph type.

``` bash
cd spark_dsg
pip install -e .
```

During installation of ROS dependencies (the last command in the above list), it is possible the container will ask you for the password; it is "guest". You may also be prompted for other configuration, like keyboard layout. Just choose whatever you usually use.

Finally, build:
```bash
cd ..
catkin build
```

You can then follow the instructions given at https://github.com/MIT-SPARK/Hydra/tree/main?tab=readme-ov-file for running the node. This will require that you download a test data set and run it using rosbag run.

### Running Hydra with the hydra_ros_netx_converter node
This section will discuss running Hydra along with the hydra_ros_netx_converter node. The latter of these nodes listens to messages containing the current dynamic scene graph, deserializes the already serialized representation of the scene graph, converts it to networkx format, filters it so that all nodes and edges have attributes which are JSON-serializable, and then writes that output to a JSON file. 

You will need three terminal instances to run everything for this example; one for hydra itself, another for playing bag bag data, and another for running the hydra_ros_netx_converter node. To open up another running instance of the docker container, open up another terminal instance and do:

``` bash
docker exec -it ros-noetic-for-hydra bash 
```

Make sure you have built all packages in the workspace, navigate to `/workspace`, and source your workspace:

``` bash
cd /workspace
source devel/setup.bash
```

You will want three terminal instances open, all three running inside the docker container. In one of them, run hydra with the uhumans2 launch file:

``` bash
roslaunch hydra_ros uhumans2.launch
```

In another terminal, run the hydra_ros_netx_converter node:

``` bash
cd /workspace
source devel/setup.bash
rosrun hydra_ros_netx_converter hydra_ros_listener.py
```

In the final terminal, run whatever bag data you want. The following assumes the uhmans2 office dataset, available here: https://web.mit.edu/sparklab/datasets/uHumans2/ and located in a directory called `data` within `/workspace`.

``` bash
cd /workspace
source devel/setup.bash
rosbag play data/uHumans2_office_s1_00h.bag
```

