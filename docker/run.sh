#!/usr/bin/env bash
ROS_DISTRO="foxy"

PIBOT_ROOT="/workspace/src/pibot_ros"
DEV_VOLUME=""
USER_COMMAND=""


if [ -z "$CONTAINER_IMAGE" ] ; then
    CONTAINER_IMAGE="pibot_ros:latest"
fi


case $1 in
    -h|-\?|--help)
        show_help    # Display a usage synopsis.
        exit
        ;;
    -d|--dev)
        DEV_VOLUME="--volume $PWD:$PIBOT_ROOT"
        ;;
esac


echo "CONTAINER_IMAGE:	$CONTAINER_IMAGE"
echo "DEV_VOLUME:	$DEV_VOLUME"
echo "USER_COMMAND:	$USER_COMMAND"

# run the container
docker run -it --rm --name pibot_ros \
	--network host \
	--privileged \
	$DEV_VOLUME \
	$CONTAINER_IMAGE $USER_COMMAND

