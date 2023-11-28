# ROS2 Exercises

This set of exercises serves to illustrate and consolidate the contents of the lectures.
You will write a set of simple ROS2 nodes, re-tracing some of the lecture examples as well as writing some new ones.

## Setup: get a ROS2 environment up and running

The first step is to get a ROS2 environment up and running.
Depending on your development machine, this can be done in various ways:

### Installing ROS2 directly on your development machine

If you are running Ubuntu 22.04 (Jammy Jellyfish) or RHEL 9, you can install ROS2 Iron according to the [official instructions][ros2-install]. Note that these installation instructions *may or may not* work on other Debian and Debian-based distributions, as well as Fedora, but may require some additional steps or adjustments. Ubuntu 22.04 is the recommended distribution for this course.

If you're running on Windows, the official instructions may work for you, but it may be easier to just run an Ubuntu 22.04 virtual machine. The performance impact of running ROS2 in a VM is negligible for the contents of these exercises.

If you're running MacOS, the best way is indeed to run an Ubuntu 22.04 virtual machine, especially when using the newer M1/M2 based Macs.

[ros2-install]: https://docs.ros.org/en/iron/Installation.html

### Using a prepared VirtualBox Machine

For the use in this course, there is a read-made VirtualBox image that you can use.
It is an Ubuntu 22.04 installation with ROS2 Iron pre-installed.
You can download the image from [here][vm-image].
Beware that this is a large file (about 8 GB) and may take a while to download.
You will of course also have to install VirtualBox on your machine.
Head to the [VirtualBox website][virtualbox] to download the installer for your operating system, or install it from your package manager.
You can then import the downloaded image into VirtualBox in the menu `File -> Import Appliance`.

Once the machine starts up, log in with the ubuntu user and the password `ubuntu`.
You will probably set the keyboard layout to your liking since you will be doing some typing.
Alternatively, you can set up a shared folder between your host machine and the virtual machine, and edit the files on your host machine.
Either way, you will execute the commands in the terminal of the virtual machine.

[vm-image]: https://muchmuch.coffee/ROS2.ova
[virtualbox]: https://www.virtualbox.org/wiki/Downloads

### Using a prepared Docker image

First, you will have to have Docker installed on your machine.
Build the docker container with the following command:

```terminal
docker build \
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    -t ros2_course .
```

This creates a docker image called `ros2_course`.
The `UID` and `GID` arguments are necessary so that your host folder will be mounted with the correct access rights in the next step.
We can now launch a container using this image with the following command:

```terminal
docker run --name=ros2_ws -it  \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /mnt/wslg:/mnt/wslg \
            -e DISPLAY \
            -e WAYLAND_DISPLAY \
            -e XDG_RUNTIME_DIR \
            -e PULSE_SERVER \
            -v $(pwd):/home/ubuntu/ros2_ws \
            ros2_course
```

This will create a docker _container_ called `ros2_ws` and start it.
The many `-v` and `-e` options are necessary to get the GUI tools to work from inside the container.
The last `-v` option mounts the current directory into the container, so that you can edit the files on your host machine:

```terminal
ls -1
slides
Dockerfile
Exercise 1.md
README.md
```

Once you exit the shell (by typing `exit` or pressing `Ctrl-D`), the container will stop.
You can now see it in its stopped mode by running `docker ps -a`: 

```terminal
CONTAINER ID   IMAGE                                             COMMAND                  CREATED              STATUS                        PORTS                                                                  NAMES
0df1bef039ff   ros2_course                                       "/ros_entrypoint.sh …"   About a minute ago   Exited (130) 11 seconds ago                                                                          ros2_ws
```

To start the container again, run `docker start ros2_ws`.
Converseley, you can stop it with `docker stop ros2_ws`.
If you want to re-create the container from scratch, you can remove it with `docker rm ros2_ws`.

To run multiple terminals inside the container, you can use the `docker exec` command:

```terminal
docker exec -it ros2_ws bash
```

This should let you run the exercises as described below.
You can verify that the basic setup works by starting the turtlesim:

```terminal
ros2 run turtlesim turtlesim_node
```

Of course, the turtle is not getting any commands, so it won't move.
But if you can see the turtle window, the setup works.

## Exercises

Follow the individual exercise instructions:

- [Exercise 1: Publisher / Subscriber](Exercise%201.md)
- Exercise 2: To come
- Exercise 3: To come