ARG BASE_IMAGE=ros:foxy-ros-base
FROM ${BASE_IMAGE}

# Shell
SHELL ["/bin/bash", "-c"]
ENV SHELL /bin/bash

#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y python3-pip python3-scipy python3-smbus

# Environment
ENV ROS_ROOT=/opt/ros/foxy
ENV WORKSPACE_ROOT=/workspace
ENV ZUMO_ROOT=${WORKSPACE_ROOT}/src/zumo_ros
ARG ROS_ENVIRONMENT=${ROS_ROOT}/setup.bash

# Workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p ${WORKSPACE_ROOT}/src

# Adafruit
RUN pip3 install Adafruit-MotorHAT Adafruit-SSD1306

# Build
COPY . ${ZUMO_ROOT}
RUN source ${ROS_ENVIRONMENT} \
 && cd ${WORKSPACE_ROOT} \
 && rosdep install -y --from-paths src --ignore-src
RUN source ${ROS_ENVIRONMENT} \
 && cd ${WORKSPACE_ROOT} \
 && colcon build --symlink-install --event-handlers console_direct+

# Scripts
COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc \
 && echo 'source ${WORKSPACE_ROOT}/install/setup.bash' >> /root/.bashrc

 ENTRYPOINT ["/ros_entrypoint.sh"]
 CMD ["bash"]
