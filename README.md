# Marslite Robot Simulator in Supermarket 

* Author: Handian Yang
* Email: ych0610765@gmail.com
* Last update: Tue, May 7, 2024

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
docker pull handianyang/marslite_simulation:v1.2.0-T265
```

### Enter the Docker container

Open a new terminal window (or press `Ctrl+Alt+t`) and reach to the root of the `marslite_simulation_ws` workspace directory (assume you have the workspace directory placed under `~` directory):
```Shell
cd ~/marslite_simulation_ws
```

#### Enter a container with the latest image tag

Run the following command to enter the Docker container based on the latest tag of `handianyang/marslite_simulation` image:
```Shell
source docker_run.sh
```

This command creates a Docker container named `marslite` and guides you in.

Specifically, the above bash script can be executed when you expect to:
- launch a new container
- enter a running container (when you work with multiple terminal windows)
- restart an exited container (when you've left the container previously)

#### (Optional) Enter a container with the specific image tag

You could specify the previous version of the image by adding version id (consecutive numbers with no dots) as parameters. 
```Shell
source docker_run.sh <tag_number>
```

For example, to invoke a container with version "v1.1.0-MPC", simply run:
```Shell
source docker_run.sh 110
```

This command creates a Docker container named `marslite_prev` and guides you in.

**[Note]** If you work with parallel terminals, ALWAYS remember to include the `<tag_number>` right behind the command (`source docker_run.sh <tag_number>`) for EVERY terminal window.

## Development

### (Recommended) Use `tmux` interface

After entering the container, simply type `tmux` to enter the tmux interface.
```Shell
tmux
```

For more `tmux` commands, please refer to the [tmux command cheat sheet](https://tmuxcheatsheet.com/).

### Build the ROS workspace

Under the root of the `marslite_simulation_ws` directory:

```Shell
catkin_make
```

(Optional) the alias:
```Shell
cm
```

### Source the ROS workspace bash script

For **EVERY** parallel terminal window, under **ANY** directory:
```Shell
source ~/marslite_simulation_ws/devel/setup.bash
```

(Optional) the alias:
```Shell
sd
```

## Clear up

### Remove the Docker container

If you somehow mess up with the existing container (e.g. having trouble `apt-get update`), or you would like to test another projects outside the current working directory, another option is to give up any changes you have made in this contaminated container.

Simply run the command **outside** the Docker container:
```Shell
docker rm marslite  # for container based on the latest tag
```
where `marslite` is the default name of the container. 

If your container was based on previous version, remember to change the container name to `marslite_prev`:
```Shell
docker rm marslite_prev  # for container based on previous tags
```



## Instructions

Note that every command should be executed **on the Docker container** rather than on your local machine.

### Launch Gazebo world

1. Launch a **Gazebo world of supermarket environment** with Marslite robot:
    ```Shell
    ### default settings
    roslaunch mars_lite_description gazebo_supermarket.launch

    ### realsense D435 camera & ros_bridge launched
    roslaunch mars_lite_description gazebo_supermarket.launch use_rosbridge:=true realsense_enabled:=true
    ```

2. Spawn a Marslite robot in an **existing Gazebo world**:
    ```Shell
    roslaunch mars_lite_description spawn_mars.launch
    ```
    **(NOTE) This SHALL NOT be launched with `gazebo_supermarket.launch`.**

### Launch the T265 camera
1. Launch the realsense T265 camera nodelet:
    ```Shell
    roslaunch realsense2_camera rs_t265.launch enable_fisheye1:=true enable_fisheye2:=true
    ```

2. Launch the realsense T265 camera nodelet with its pose displayed in RVIZ:
    ```Shell
    roslaunch realsense2_camera demo_t265.launch
    ```
    **(NOTE) <br>
    (1) This SHALL NOT be launched with `rs_t265.launch`.<br>
    (2) The `demo_t265.launch` does not contain `enable_fisheye1` and `enable_fisheye2` parameters.**

### Navigation features

1. Launch **SLAM** using 'gmapping' method:
    ```Shell
    roslaunch marslite_navigation slam_gmapping.launch
    ```

2. Launch **navigation** with  A* and DWA:
    ```Shell
    roslaunch marslite_navigation navigation.launch
    ```

3. Directly drive the robot with keyboard inputs:
    ```Shell
    roslaunch marslite_navigation teleop_keyboard.launch
    ```

4. Directly drive the robot with joystick inputs:
    ```Shell
    roslaunch marslite_navigation teleop_joystick.launch
    ```

5. Drive the robot with shared autonomy with keyboard inputs
(`navigation.launch` is required to launch)
    ```Shell
    roslaunch marslite_navigation shared_control_keyboard.launch
    ```

6. Drive the robot with shared autonomy with joystick inputs
(`navigation.launch` is required to launch)
    ```Shell
    roslaunch marslite_navigation shared_control_joystick.launch
    ```


### Robotic arm control

1. Launch the Moveit! control interface
    ```Shell
    roslaunch mars_lite_moveit_config mars_lite_moveit_planning_execution_gz.launch
    ```

2. Trajectory planning using MPC model
    ```Shell
    roslaunch marslite_control test_default_pose_planning.launch
    ```


## Known issues

### Build-related

#### Warnings
- `Eigen` is deprecated

### Robot-related

#### Errors
- **"No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/..."**

<!-- #### warnings
- **"TF_REPEATED_DATA ignoring data with redundant timestamp..."** -->


<!-- ### Gazebo-related

#### errors
- Press "Reset model pose" would result in unexpected force applied on the marslite.
- Node::Advertise(): Error advertising topic [/shelf_01_0/joint_cmd]. Did you forget to start the discovery service? [Err] [JointControlWidget.cc:393] Error advertising topic [/room_01/joint_cmd] -->
