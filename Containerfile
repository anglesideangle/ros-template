# Set up ROS2 container for diesl, and configure Zenoh for the RMW
ARG ROS_DISTRO=jazzy TAG=latest REG=registry.gitlab.sitcore.net/haiisw/diesl



###########################################################################
### BASE IMAGE ###
###########################################################################

# The base layer only installs dependencies, and does not build any code
# It installs a full ros environment

# You can either user the diesl base image or the official ROS base image
# FROM ${REG}:diesl-base:${TAG} AS base
FROM ros:${ROS_DISTRO}-ros-base  AS base

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 OPAL_PREFIX= ROS_DISTRO=${ROS_DISTRO}

# Install packages
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  # ros
  ros-dev-tools \
  ros-${ROS_DISTRO}-ros-base \
  git \
  python3-pip \
  pip \
  python3-rosdep \ 
  wget \
  unzip \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-launch \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-ament-cmake \ 
  ros-${ROS_DISTRO}-ament-cmake-core \ 
  ros-${ROS_DISTRO}-ament-cmake-python \
  # alternative RMW packages:
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  # Add additional depencies here as needed, these are for the example
  ros-jazzy-example-interfaces \
  ros-jazzy-demo-nodes-cpp \
  python3-pytest \
  python3-numpy

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CONFIG_URI=/CUSTOM_RMW_ZENOH_ROUTER_CONFIG.json5 ZENOH_SESSION_CONFIG_URI=/CUSTOM_RMW_ZENOH_SESSION_CONFIG.json5
# expose ports used for Zenoh
EXPOSE 7447/tcp
EXPOSE 8000/tcp
EXPOSE 7447/udp
EXPOSE 7446/udp

COPY root/ /

RUN chmod +x /entrypoint.bash
ENTRYPOINT [ "/entrypoint.bash" ]

STOPSIGNAL SIGINT


###########################################################################
### BUILD IMAGE ###
###########################################################################

# Full image with all the functionality, built on top of the base image
FROM base AS build

# Copy in workspace
COPY . /workspace
WORKDIR /workspace

# Actually build package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

STOPSIGNAL SIGINT

###########################################################################
### FINAL IMAGE ###
###########################################################################

# Stripped down image with only what is needed to run the code
FROM ros:${ROS_DISTRO}-ros-core AS final

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-rmw-zenoh-cpp \
# Add additional depencies here as needed, these are examples
  ros-jazzy-example-interfaces \
  ros-jazzy-demo-nodes-cpp \
  python3-pytest \
  python3-numpy \
  # and clean up apt cache
  && apt-get autoclean \
  && rm -rf /var/lib/apt/lists/*

COPY --from=build /workspace/build /workspace/build
COPY --from=build /workspace/install /workspace/install

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 OPAL_PREFIX= 
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CONFIG_URI=/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 ZENOH_SESSION_CONFIG_URI=/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5
# expose ports used for Zenoh
EXPOSE 7447/tcp
EXPOSE 8000/tcp
EXPOSE 7447/udp
EXPOSE 7446/udp

COPY root/ /

RUN chmod +x /entrypoint.bash
ENTRYPOINT [ "/entrypoint.bash" ]

# Example of run command, in this case startng a zenoh router node
# CMD ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]

STOPSIGNAL SIGINT



###########################################################################
### DEV IMAGE ###
###########################################################################

# Dev environment with useful tools
FROM base AS dev

# Install tools
RUN apt-get install -y software-properties-common \
  && add-apt-repository ppa:maveonair/helix-editor \
  && apt-get install -y vim \
    tmux \
    helix \
    clang \
    clang-tools \
    python3-pylsp \
    bash-completion

# Make bash prompt nicer
RUN echo 'PS1="🤖  \[\e[38;5;130m\]\u@\h \[\e[34m\]\w\[\e[0m\] $ "' >> /root/.bashrc

