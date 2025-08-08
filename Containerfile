ARG ROS_DISTRO

# The base layer only installs dependencies, and does not build any code
# It installs a full ros environment
FROM docker.io/osrf/ros:${ROS_DISTRO}-desktop-full AS base

ENV DEBIAN_FRONTEND=noninteractive

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
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-launch \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-ament-cmake \ 
  ros-${ROS_DISTRO}-ament-cmake-core \ 
  ros-${ROS_DISTRO}-ament-cmake-python
  # TODO add your dependencies here!

# Set up the entrypoint
RUN cat <<EOF > /entrypoint.bash
#!/usr/bin/env bash
source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

exec "\$@"
exec bash
EOF

RUN chmod +x /entrypoint.bash
ENTRYPOINT [ "/entrypoint.bash" ]

# The build layer (which should probably be renamed), actually builds everything.
FROM base AS build

# Copy in workspace
COPY . /workspace
WORKDIR /workspace

# Actually build package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Start from mininal image for final
FROM docker.io/library/ros:${ROS_DISTRO}-ros-core AS final

# TODO uncomment this and add runtime dependencies
# RUN apt-get update \
#   && apt-get install -y --no-install-recommends \
#     my-package
#   && rm -rf /var/lib/apt/lists/*

COPY --from=build /workspace/build /workspace/build
COPY --from=build /workspace/install /workspace/install

CMD [ "bash" ] # TODO your command here!!

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

