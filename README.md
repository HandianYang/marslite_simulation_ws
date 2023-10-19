# Marslite Robot Simulator in Supermarket 

* Author: Handian Yang
* Email: ych0610765@gmail.com
* Last update: Thr, Oct 19, 2023

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
docker pull handianyang/marslite_simulation:v1.0.0-Base
```

<!--
Of course you could check [the tags of my image](https://hub.docker.com/repository/docker/handianyang/marslite_simulation/general) to specify older versions, but the complete features of the system are not guarenteed. For example, to obtain the base image (the most primitive one):
```Shell
docker pull handianyang/marslite_simulation:v1.0.0-Base
```
-->

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

Run the following command to enter the Docker container based on `handianyang/marslite_simulation` image:
```Shell
source docker_run.sh
```

Specifically, the above bash script can be executed when you expect to:
- launch a new container
- enter a running container (when you work with multiple terminal windows)
- restart an exited container (when you've left the container previously)

If you work with parallel terminals, just repeat the above two instructions.


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

If you somehow mess up with the existing container (e.g. having trouble `apt-get update`), another option is to give up any changes you have made in this contaminated container.

Simply run the command **outside** the Docker container:
```Shell
docker rm marslite
```
where `marslite` is the default alias of your container.



## Instructions

Note that every command should be executed **on the Docker container** rather than on your local machine.

### Launch Gazebo world

1. Launch a **Gazebo world of supermarket environment** with Marslite robot:
```Shell
roslaunch mars_lite_description gazebo_supermarket.launch
```

2. Spawn a Marslite robot in an **existing Gazebo world**:
```Shell
roslaunch mars_lite_description spawn_mars.launch
```

### Navigation features

1. Launch **SLAM** using 'gmapping' method:
```Shell
roslaunch marslite_navigation slam_gmapping.launch
```

2. Launch **navigation with  A* and DWA**:
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


### Robotic arm control

1. default Moveit! control interface
```Shell
roslaunch mars_lite_moveit_config mars_lite_moveit_planning_execution_gz.launch
```


## Known issues

### Robot-related

#### errors
- **"No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/..."**

#### warnings
- **"TF_REPEATED_DATA ignoring data with redundant timestamp..."**


### Gazebo-related

#### errors
- Press "Reset model pose" would result in unexpected force applied on the marslite.
- Node::Advertise(): Error advertising topic [/shelf_01_0/joint_cmd]. Did you forget to start the discovery service? [Err] [JointControlWidget.cc:393] Error advertising topic [/room_01/joint_cmd]
