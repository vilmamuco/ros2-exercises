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

### Using a prepared Docker image and docker-compose

TODO

## Exercises

Follow the individual exercise instructions:

- [Exercise 1: Publisher / Subscriber](Exercise%201.md)
- Exercise 2: To come
- Exercise 3: To come