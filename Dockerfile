FROM ros:jazzy-ros-base

# not expecting a cache mechanism like the one in buildx, the base image includes
# this config file that essentially purges the cache on every install operation
# which is why we have to remove this config to take advantage of the host's cache
# with --mount=cache further down
# If you don't have buildx, you can still use this Dockerfile, but you'll have to
# remove the RUN rm line below as well as the --mount=cache lines
RUN rm /etc/apt/apt.conf.d/docker-clean

RUN \
    # holds the package _indexes_ (used for apt update)
    --mount=type=cache,target=/var/lib/apt/lists \
    # holds the package _contents_ (used for apt install)
    --mount=type=cache,target=/var/cache/apt/archives \
    apt update && \
    apt install -y --no-install-recommends \
    ros-jazzy-turtlesim

# for nvidia graphics. comment out if this causes issues for you
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND=noninteractive

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ubuntu

RUN useradd -u $UID -g $GID --system --create-home --no-log-init ubuntu
# set password of ubuntu to "ubuntu"
RUN echo "ubuntu:ubuntu" | chpasswd && \
    usermod -aG sudo ubuntu && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN chsh -s /bin/bash ubuntu
USER ubuntu

RUN mkdir /home/ubuntu/ros2_ws
WORKDIR "/home/ubuntu/ros2_ws"

SHELL ["/bin/bash", "-c"]    # tells docker to use the bash shell

# source the workspace activation script
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc

VOLUME "/home/ubuntu/ros2_ws"
