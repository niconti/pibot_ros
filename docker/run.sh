#!/usr/bin/env bash
ROS_DISTRO="foxy"
PIBOT_ROOT="/workspace/src/pibot_ros"

case $1 in
    -h|-\?|--help)
        show_help    # Display a usage synopsis.
        exit
        ;;
    -d|--dev)
        DEV_VOLUME="--volume $PWD:$PIBOT_ROOT"
        ;;
esac


if [ -z "$CONTAINER_IMAGE" ] ; then
    CONTAINER_IMAGE="pibot_ros:latest"
fi

# run the container
docker run -it --rm --name pibot_ros \
    --network host \
    --privileged \
    $MOUNTS $CONTAINER_IMAGE $USER_COMMAND
