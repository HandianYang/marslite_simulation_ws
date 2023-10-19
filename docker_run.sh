#!/usr/bin/env sh
xhost +local:docker

# Setup specific docker image and tag
DOCKER_IMAGE="handianyang/marslite_simulation"
DOCKER_LATEST_TAG="v1.1.0-MPC"
CONTAINER_NAME="marslite"

# Setup the style of color
COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[0;33m'
COLOR_NC='\033[0m'

#
# Check the command 'nvidia-docker' existed or not
#
ret_code="$(command -v nvidia-docker)"
if [ -z "$ret_code" ]; then
    if [ -z "$(command -v nvidia-container-cli)" ]; then
        NVIDIA_SUPPORT_OPTION="--gpus all "
    else
        NVIDIA_SUPPORT_OPTION=""
    fi
    DOCKER_CMD="docker"
else
    DOCKER_CMD="nvidia-docker"
    NVIDIA_SUPPORT_OPTION=""
fi

#
# Specify docker image tag
#
run_docker=true
if [ $# -gt 0 ]; then
    if [ "$1" == "base" ]; then
	DOCKER_TAG="v1.0.0-Base"
    else
    	DOCKER_TAG=$DOCKER_LATEST_TAG
    fi
else
    DOCKER_TAG=$DOCKER_LATEST_TAG
fi


# Check if we need to run docker
if $run_docker; then
    echo "[IMAGE:TAG] $DOCKER_IMAGE:$DOCKER_TAG"

    # Find current directory and transfer it to container directory for Docker
    current_dir="$(pwd)"
    host_dir="${HOME}/"
    container_dir="/home/developer/"
    goal_dir=${current_dir//$host_dir/$container_dir}

    # Check if the Docker container exists
    if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
        # The container exists, check if it's running or stopped
        if [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME)" == "true" ]; then
            echo "The \"$CONTAINER_NAME\" container is RUNNING, so enter it."
            docker exec -it ${CONTAINER_NAME} bash
        else
            echo "The \"$CONTAINER_NAME\" container is STOPPED, so restart it."
            docker start -ai ${CONTAINER_NAME}
        fi
    else
        echo "The \"$CONTAINER_NAME\" container DOES NOT EXIST, so create a new container."
        ${DOCKER_CMD} run -it --privileged ${NVIDIA_SUPPORT_OPTION} \
            --name ${CONTAINER_NAME} \
            --net=host \
            --env DISPLAY=$DISPLAY \
            --env QT_X11_NO_MITSHM=1 \
            -v /dev:/dev \
            -v /etc/localtime:/etc/localtime:ro \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
            -v /dev/bus/usb:/dev/bus/usb \
            -v ${current_dir}:${goal_dir} \
            -w ${goal_dir} \
            --device=/dev/snd \
            --device=/dev/dri \
            --device=/dev/nvhost-ctrl \
            --device=/dev/nvhost-ctrl-gpu \
            --device=/dev/nvhost-prof-gpu \
            --device=/dev/nvmap \
            --device=/dev/nvhost-gpu \
            --device=/dev/nvhost-as-gpu \
            ${DOCKER_IMAGE}:${DOCKER_TAG}  
    fi
fi

