#!/usr/bin/env bash
ROS_DISTRO="foxy"

ZUMO_ROOT="/workspace/src/zumo_ros"
DEV_VOLUME=""
USER_COMMAND=""

if [ -z "$CONTAINER_IMAGE" ] ; then
    CONTAINER_IMAGE="zumo_ros:latest"
fi

show_help() {
    echo " "
    echo "usage: Starts the Docker container and runs a user-specified command"
    echo " "
    echo "   ./docker/run.sh --ros ROS_DISTRO "
    echo "                   --container DOCKER_IMAGE"
    echo "                   --volume HOST_DIR:MOUNT_DIR"
    echo "                   --run RUN_COMMAND"
    echo " "
    echo "args:"
    echo " "
    echo "   --help                       Show this help text and quit"
    echo " "
    echo "   --ros ROS_DISTRO  ROS distro to use:  eloquent, foxy (default is foxy)"
    echo "                     This option sets the container image to use."
    echo " "
    echo "   -c, --container DOCKER_IMAGE  Manually specify the name/tag of the Docker"
    echo "                                 container to use, overriding --ros option."
    echo " "
    echo "   -d, --dev  Runs the container in development mode, where the source"
    echo "              files are mounted into the container dynamically, so they"
    echo "              can more easily be edited from the host machine."
    echo " "
    echo "   -v, --volume HOST_DIR:MOUNT_DIR  Mount a path from the host system into"
    echo "                                    the container.  Should be specified as:"
    echo " "
    echo "                                       -v /my/host/path:/my/container/path"
    echo " "
    echo "                                    (these should be absolute paths)"
    echo " "
    echo "   -r, --run RUN_COMMAND  Command to run once the container is started."
    echo "                          Note that this argument must be invoked last,"
    echo "                          as all further arguments will form the command."
    echo "                          If no run command is specified, an interactive"
    echo "                          terminal into the container will be provided."
    echo " "
}

while :; do
    case $1 in
        -h|-\?|--help)        
            show_help                   # Display a usage synopsis.
            exit
            ;;
        -c|--container)       
            if [ "$2" ]; then           # Takes an option argument; ensure it has been specified.
                CONTAINER_IMAGE=$2
                shift
            else
                echo 'ERROR: "--container" requires a non-empty option argument.'
                exit 1
            fi
            ;;
        -d|--dev)
            DEV_VOLUME="--volume $PWD:$ZUMO_ROOT"
            ;;
        -r|--run)
            if [ "$2" ] ; then          # Takes an option argument; ensure it has been specified.
                shift
                USER_COMMAND="$@"
            else
                echo 'ERROR: "--run" requires a non-empty option argument.'
                exit 1
            fi
            ;;
        *)                              # Default case: No more options, so break out of the loop.
            break
    esac
    shift
done

echo "CONTAINER_IMAGE:  $CONTAINER_IMAGE"
echo "DEV_VOLUME:       $DEV_VOLUME"
echo "USER_COMMAND:     $USER_COMMAND"

# run the container
docker run -it --rm --name zumo_ros \
	--network host \
	--privileged \
    --volume /dev:/dev \
	$DEV_VOLUME \
	$CONTAINER_IMAGE $USER_COMMAND
