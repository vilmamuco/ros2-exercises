# ROS2 Exercises

This set of exercises serves to illustrate and consolidate the contents of the lectures.
You will write a set of simple ROS2 nodes, re-tracing some of the lecture examples as well as writing some new ones.

## Setup: get a ROS2 environment up and running

The first step is to get a ROS2 environment up and running.
Depending on your development machine, this can be done in various ways.

### On your development machine: installing ROS2 directly

If you are running Ubuntu 24.04 (Noble Numbat) or RHEL 9, you can install ROS2 Jazzy according to the [official instructions][ros2-install]. Note that these installation instructions _may or may not_ work on other Debian and Debian-based distributions, as well as Fedora, but may require some additional steps or adjustments. Ubuntu 22.04 is the recommended distribution for this course.

If you're running on Windows, the official instructions may work for you, but it may be easier to just run an Ubuntu 22.04 virtual machine. The performance impact of running ROS2 in a VM is negligible for the contents of these exercises.

If you're running MacOS, the best way is indeed to run an Ubuntu 22.04 virtual machine, especially when using the newer M1/M2 based Macs.

[ros2-install]: https://docs.ros.org/en/jazzy/Installation.html

If you run ROS2 on your development machine, you can skip the next section and go straight to the exercises.
Make sure to adjust the workspace path (`/home/ubuntu/ros2_ws`) in the exercise instructions to match your setup.

### In the lab: loading the prepared Docker image

We prepared you a docker _image_ that you can load and launch to create a _container_.
Once you have the file's path, execute:

```terminal
docker load -i <path-to-image-file>
8ceb9643fb36: Loading layer [==================================================>]  80.37MB/80.37MB
[...]
fcd43f8e7c85: Loading layer [==================================================>]  6.656kB/6.656kB
Loaded image: ros2_course:latest
```

### On your machine: building from the Dockerfile

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

## Running the container

This is of course only necessary when using docker.
To run the container but keep your code between runs, we'll have to create a folder on your host machine that will be mounted into the container:

```terminal
mkdir -p ~/ros2_ws
# save this path for later
export ROS2_WS_PATH=~/ros2_ws
```

Next we'll have to start the container.

```terminal
docker run --name=ros2_ws -it  \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /mnt/wslg:/mnt/wslg \
            -e DISPLAY \
            -e WAYLAND_DISPLAY \
            -e XDG_RUNTIME_DIR \
            -e PULSE_SERVER \
            -v ${ROS2_WS_PATH}:/home/ubuntu/ros2_ws \
            --device=/dev/dri:/dev/dri \
            ros2_course
```

**Note:** the `--device=/dev/dri:/dev/dri` option is used for AMD or Intel GPUs. For NVIDIA GPUs, you will have to use `--gpus all` instead.

This will create a docker _container_ called `ros2_ws` and start it.
The many `-v` and `-e` options are necessary to get the GUI tools to work from inside the container.
The last `-v` option mounts the workspace folder we created into the container, so that you can edit the files on your host machine.
The container will start a shell inside the workspace folder, which is currently empty.

Once you exit the shell (by typing `exit` or pressing `Ctrl-D`), the container will stop.
You can now see it in its stopped mode by running `docker ps -a`:

```terminal
CONTAINER ID   IMAGE                                             COMMAND                  CREATED              STATUS                        PORTS                                                                  NAMES
0df1bef039ff   ros2_course                                       "/ros_entrypoint.sh …"   About a minute ago   Exited (130) 11 seconds ago                                                                          ros2_ws
```

To start the container again, run `docker start ros2_ws`.
Converseley, you can stop it with `docker stop ros2_ws`.
If you want to re-create the container from scratch, you can remove it with `docker rm ros2_ws`.

To run multiple terminals inside the container, you can use the `docker exec` command.
On your host machine, open another terminal and run:

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

A word of advice, since this is a common mistake: don't forget to source your own workspace.
After building your own packages, you will have to source your workspace again:

```terminal
source install/setup.bash
```

The global ros2 workspace (the "underlay") is already sourced for you inside the Dockerfile.

You are now ready to move on to the exercises.

## Exercises

Follow the individual exercise instructions:

- [Exercise 1: Topics](Exercise%201.md)
- [Exercise 2: Services](Exercise%202.md)
- [Exercise 3: Actions](Exercise%203.md)
