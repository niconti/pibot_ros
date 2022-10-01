#!/usr/bin/env bash
ROS_DISTRO=foxy
BASE_IMAGE=ros:foxy-ros-base

set -e

container_image=pibot_ros

echo "Building $container_image ..."
echo "BASE_IMAGE=$BASE_IMAGE"

docker build -t $container_image \
	--build-arg BASE_IMAGE=$BASE_IMAGE \
	.

