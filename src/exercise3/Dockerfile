FROM ros:jazzy-ros-base

# not expecting a cache mechanism like the one in buildx, the base image includes
# this config file that essentially purges the cache on every install operation
# which is why we have to remove this config to take advantage of the host's cache
# with --mount=cache further down
RUN rm /etc/apt/apt.conf.d/docker-clean

ENV DEBIAN_FRONTEND noninteractive


#--------------------------------------------Gazebo---------------------------------------------------
RUN \
  # holds the package _indexes_ (used for apt update)
  --mount=type=cache,target=/var/lib/apt/lists \
  # holds the package _contents_ (used for apt install)
  --mount=type=cache,target=/var/cache/apt/archives \
  apt update && \
  apt install -y --no-install-recommends \
  build-essential \
  keyboard-configuration \
  locales \
  tzdata \
  git \
  cmake \
  wget \
  sudo \
  gazebo \
  ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs

ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
  locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# create new user
RUN adduser --disabled-password \
  --gecos '' docker

#  Add new user docker to sudo group
RUN adduser docker sudo
RUN adduser docker dialout

# Ensure sudo group users are not asked for a password when using
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
  /etc/sudoers

# now we can set USER to the user we just created
USER docker

ENV DEBIAN_FRONTEND noninteractive

# download for freebuoyancy_gazebo ------------------------------------------------------------------
WORKDIR /home/docker/
RUN git clone https://github.com/bluerobotics/freebuoyancy_gazebo
WORKDIR /home/docker/freebuoyancy_gazebo
# libgazebo11-dev not sure about the version number
RUN USER=docker sudo apt install libgazebo11 -y
RUN ["/bin/bash","-c","source gazebo.sh "]
RUN /bin/bash -c "sudo apt update"
RUN /bin/bash -c "sudo apt install liburdfdom-dev -y"
RUN ["/bin/bash","-c","mkdir build"]
RUN ["/bin/bash","-c","sudo apt update && sudo apt install ros-jazzy-gazebo-ros-pkgs -y"]
WORKDIR /home/docker/freebuoyancy_gazebo/build/
RUN ["/bin/bash","-c"," cmake .. && make"]
RUN ["/bin/bash","-c","sudo make install"]

#-----------------------------------------Ardupilot---------------------------------------------------

WORKDIR /home/docker/
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/docker/ardupilot
RUN git checkout 94ba4ec
RUN git submodule update --init --recursive

# RUN git config --global --add safe.directory /ardupilot
WORKDIR /home/docker/ardupilot/Tools/environment_install/
RUN rm install-prereqs-ubuntu.sh
RUN wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/environment_install/install-prereqs-ubuntu.sh
WORKDIR /home/docker/ardupilot
RUN chmod +x Tools/environment_install/install-prereqs-ubuntu.sh
RUN USER=docker Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN . ~/.profile

RUN sudo pip3 install -U mavproxy PyYAML

ENV PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
ENV PATH=$PATH:$HOME/docker/ardupilot/Tools/autotest
ENV PATH=/usr/lib/ccache:$PATH

WORKDIR /home/docker/ardupilot
RUN ["/bin/bash","-c","./waf configure && make sub"]
WORKDIR /home/docker/ardupilot/ArduSub

# ardupilot gazebo plugin ---------------------------------------------------------------------------
WORKDIR /home/docker/
#RUN git clone https://github.com/ArduPilot/ardupilot_gazebo /home/docker/ardupilot_gazebo
RUN git clone https://github.com/vilmamuco/ardupilot_gazebo /home/docker/ardupilot_gazebo
WORKDIR /home/docker/ardupilot_gazebo
RUN git checkout add_link
#RUN git checkout ros2
RUN mkdir build
WORKDIR /home/docker/ardupilot_gazebo/build
RUN cmake  ..
RUN make -j4
RUN sudo make install
# set path of gazebo models
RUN echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
# set path of gazebo worlds
RUN echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

# download BlueROV2 gazebo model ---------------------------------------------------------------------
RUN git clone https://github.com/vilmamuco/bluerov_ros_playground /home/docker/bluerov_ros_playground
WORKDIR /home/docker/bluerov_ros_playground
# RUN source gazebo.sh

# dowload database for mavros ---------------------------------------------------------------------
WORKDIR /home/docker/
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
RUN sudo bash ./install_geographiclib_datasets.sh
RUN pwd
WORKDIR /home/docker/freebuoyancy_gazebo/
RUN ["/bin/bash","-c","source gazebo.sh "]
CMD  /bin/bash
