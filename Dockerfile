ARG BASE_IMAGE=ros:foxy-ros-base
FROM ${BASE_IMAGE}

# Shell
SHELL ["/bin/bash", "-c"]
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive

# Environment
ENV ROS_ROOT=/opt/ros/foxy
ENV WORKSPACE_ROOT=/workspace
ENV PIBOT_ROOT=${WORKSPACE_ROOT}/src/pibot_ros
ARG ROS_ENVIRONMENT=${ROS_ROOT}/setup.bash

# Workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p ${WORKSPACE_ROOT}/src

# Build
COPY . ${PIBOT_ROOT}/
RUN source ${ROS_ENVIRONMENT} \
 && cd ${WORKSPACE_ROOT} \
 && colcon build --symlink-install --event-handlers console_direct+

# Scripts
COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc \
 && echo 'source ${WORKSPACE_ROOT}/install/local_setup.bash' >> /root/.bashrc

 ENTRYPOINT ["/ros_entrypoint.sh"]
 CMD ["bash"]
