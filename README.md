# Marslite Robot Simulator in Supermarket 

* Author: Handian Yang
* Email: ych0610765@gmail.com
* Last update: Thu, Feb 8, 2024

## Prerequisites

### Hardware (PC)
- NVIDIA GPU (compatible with NVIDIA GPU driver 515 version)

### Software
- NVIDIA-driver-515
- nvidia-docker2
- docker engine

## Setup

### Install Docker engine

The required packages and tools of this simulator system could be easily accessed by specifying the Docker image. With the usage of Docker images, users do not need to worry about any possibility to encounter problems related to incompatible software tools. To completely understand what Docker is, please refer to [this webpage](https://docs.docker.com/get-started/).

If you have not installed Docker engine on your Ubuntu desktop, please click [this](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) to follow the instructions in the official Docker documents.

### Pull the specified Docker image from Docker Hub

Docker Hub helps you create, manage, and deliver your team's container applications. Here you can find and use various kinds of Docker images, or create your own Docker images.

To utilize the simulator system, users are required to download my designed image. Run the following command to obtain the latest version of my Docker image (click [here](https://hub.docker.com/repository/docker/handianyang/marslite_simulation/general) to check the tagname):
```Shell
docker pull handianyang/marslite_simulation:v1.1.2-MPC-Bugfix
```

**[Optional]** Of course you could check [the tags of my image](https://hub.docker.com/repository/docker/handianyang/marslite_simulation/general) to specify older versions, but the complete features of the system are not guarenteed. For example, to obtain the base image (the most primitive one):
```Shell
docker pull handianyang/marslite_simulation:v1.0.0-Base
```

## Usage

### Download the GitHub repository

It's recommended to download it under `~` directory:
```Shell
cd ~
git clone https://github.com/HandianYang/marslite_simulation_ws.git
```

### Enter the Docker container

Open a new terminal window (or press `Ctrl+Alt+t`) and reach to the root of the `marslite_simulation_ws` workspace directory (assume you have the workspace directory placed under `~` directory):
```Shell
cd ~/marslite_simulation_ws
```

#### 1. Enter a container with the latest image tag

Run the following command to enter the Docker container based on the latest tag of `handianyang/marslite_simulation` image:
```Shell
source docker_run.sh
```

This command creates a Docker container named `marslite` and guides you in.

Specifically, the above bash script can be executed when you expect to:
- launch a new container
- enter a running container (when you work with multiple terminal windows)
- restart an exited container (when you've left the container previously)

#### 2. Enter a container with the specific image tag

You could specify the previous version of the image by adding version id (only consecutive numbers with no dots) as parameters. 
```Shell
source docker_run.sh <tag_number>
```

For example, to invoke a container with version "v1.1.0-MPC", simply run:
```Shell
source docker_run.sh 110
```

This command creates a Docker container named `marslite_prev` and guides you in.

**[Note]** If you work with parallel terminals, ALWAYS remember to include the `<tag_number>` right behind the command (`source docker_run.sh <tag_number>`) for EVERY terminal window.

### Build the ROS workspace

After entering the Docker container, directly build the ROS workspace using `catkin_make` under the root directory of `marslite_simulation_ws` workspace:
```Shell
catkin_make
```

Or using the alias we provided to speed up your typing:
```Shell
cm
### The above is the same as ...
###   catkin_make
```

If you've encountered any warnings or errors while building the workspace, please feel free to report an issue!


### Source the workspace bash script

To let the changes of the workspace's environment variables take effect, remember to source the workspace bash script:
```Shell
source ~/marslite_simulation_ws/devel/setup.bash
```

Note that the command should be **executed in every terminal window**. Also you should change the pathname if you had placed the workspace directory at any other places.

Since typing the same command into every terminal window is too trivial for multiple-terminal-window developers, we also offer an alias `sd` for this command:
```Shell
sd
### The above is the same as ...
###   `source ~/marslite_simulation_ws/devel/setup.bash`
### The pathname should also be changed if necessary.
```

From now on, you can begin your development! For detailed features and their corresponding commands, please refer to the Instruction section below.


### Remove the Docker container

If you somehow mess up with the existing container (e.g. having trouble `apt-get update`), or you would like to test another projects outside the current working directory, another option is to give up any changes you have made in this contaminated container.

Simply run the command **outside** the Docker container:
```Shell
docker rm marslite  # for container based on latest tag
```
where `marslite` is the default name of the container. Or if your container was based on previous version, remember to change the container name to `marslite_test`:
```Shell
docker rm marslite_prev  # for container based on previous tag
```



## Instructions

Note that every command should be executed **on the Docker container** rather than on your local machine.

### Launch Gazebo world

1. Launch a **Gazebo world of supermarket environment** with Marslite robot:
```Shell
roslaunch mars_lite_description gazebo_supermarket.launch
```

2. **(Optional)** Spawn a Marslite robot in an **existing Gazebo world**:
```Shell
roslaunch mars_lite_description spawn_mars.launch
```

### Navigation features

1. Launch **SLAM** using 'gmapping' method:
```Shell
roslaunch marslite_navigation slam_gmapping.launch
```

2. Launch **navigation** with  A* and DWA:
```Shell
roslaunch marslite_navigation navigation.launch
```

3. Directly control the robot with keyboard inputs:
```Shell
roslaunch marslite_navigation teleop_keyboard.launch
```

4. Directly control the robot with joystick inputs:
```Shell
roslaunch marslite_navigation teleop_joystick.launch
```

<!-- ### Obstacle avoidance

1. Publish **"static virtual zone (SVZ)"** topic:
```Shell
rosrun marslite_shared_control static_virtual_zone
```

2. Publish **"deformable virtual zone (DVZ)"** topic:
```Shell
rosrun marslite_shared_control deformable_virtual_zone
``` -->


### Robotic arm control

1. default Moveit! control interface
```Shell
roslaunch mars_lite_moveit_config mars_lite_moveit_planning_execution_gz.launch
```


## Known issues

### System-related

#### Warnings
- CMake warning: `Eigen` is deprecated
- Build warning: `_bool OsqpEigen::Solver::solve()_` is deprecated

### Robot-related

#### Errors
- **"No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/..."**

#### warnings
- **"TF_REPEATED_DATA ignoring data with redundant timestamp..."**


### Gazebo-related

#### errors
- Press "Reset model pose" would result in unexpected force applied on the marslite.
- Node::Advertise(): Error advertising topic [/shelf_01_0/joint_cmd]. Did you forget to start the discovery service? [Err] [JointControlWidget.cc:393] Error advertising topic [/room_01/joint_cmd]
