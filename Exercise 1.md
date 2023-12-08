# Exercise 1: Topics

In the first exercise, you will write two simple ROS2 nodes, a publisher and a subscriber.
The publisher will publish a simple message type, and the subscriber will subscribe to that message type and print the received messages to the console.

More than anything else, this serves to make sure your toolchain is set up correctly and you can build and run ROS2 nodes.

## Creating a workspace and package

First, we'll create a package for our publisher and subscriber nodes.
We'll execute the following commands inside the docker container.
As a reminder, launching the container is done with the following command:

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

Again, use `--gpus all` instead of `--device=/dev/dri:/dev/dri` if you have an NVIDIA GPU.

Once you're inside the docker container, create the new package:

```terminal
cd ~/ros2_ws/src

ros2 pkg create \
    --build-type ament_python \
    --node-name circle_controller p_controller \
    --license Apache-2.0
```

Of course, pick any license you see fit.
The next step is to build the package.
Since we're using python, which doesn't need compilation, we can tell colcon to "symlink" the package into the workspace, which means that we don't have to rebuild the package every time we make a change.

```terminal
colcon build --packages-select p_controller --symlink-install
```

Note that this will print a bit of output about easy_install being deprecated.
Indeed, modern python development relies on newer tools like [poetry][poetry] but ROS2 is still using the older tools in the package templates, so we'll not concern ourselves with that for now.

[poetry]: https://python-poetry.org/

Once the build finishes, we'll source the workspace so that ROS2 can find our package.

```terminal
source install/setup.bash
```

The last thing we need to do is to modify `package.xml` and `setup.py` to set the `description`, `maintainer` and `license` fields.
Make sure the information between the two files matches exactly.

## Writing the publisher

Now, we'll write the publisher node.
The file `~/ros2_ws/src/p_controller/p_controller/circle_controller.py` contains a mostly empty python file.

In this file, create a class `CircleController` that inherits from `rclpy.node.Node`.
In the constructor, call the super constructor with the node name `circle_controller`.
Then, create a timer that calls a callback function every 1 second.

The callback function should publish a `geometry_msgs.msg.Twist` message on the topic `turtle1/cmd_vel`.

The message should have a linear velocity of 1 m/s and an angular velocity of 1 rad/s.

Once you have the node finished, fill in the `main` function so that it creates a `CircleController` node and "spins" it.
You will have to first initialize the ROS2 node system with `rclpy.init()`, create the node, and then call `rclpy.spin()` to start the node.

If you get stuck, check the lecture slides.

Since you used the `Twist` message type, as well as `rclpy`, you'll have to add these to the dependencies in `package.xml`:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Since you generated the package structure with the call to `ros2 pkg create`, the `setup.py` file should already contain the correct entry point:

```python
entry_points={
    'console_scripts': [
        # before the "=" we have the node name
        # after the "=" we have <module name>:<function name>
        'circle_controller = p_controller.circle_controller:main'
    ],
},
```

## Launching both nodes manually

For the sake of exercise, we'll launch both our own node and turtlesim manually, without using a launch file.

Open two terminals side by side, and in each, attach to your docker container, and then source the workspace:

```terminal
# on the host machine
docker exec -it ros2_ws bash
```

```terminal
# inside docker
cd ~/ros2_ws/src
source install/setup.bash
```

In the first terminal, run the publisher node:

```terminal
ros2 run p_controller circle_controller
```

It will not print anything, but it will publish messages to the `turtle1/cmd_vel` topic.

In the second terminal, run the subscriber node:

```terminal
ros2 run turtlesim turtlesim_node
```

This should show the turtlesim window, and the turtle should be moving in a circle.
You can stop the nodes by hitting `Ctrl+C` on the keyboard.

## Launching both nodes with a launch file

Now, we'll create a launch file that launches both nodes at the same time.

Inside the `p_controller` folder, create a new folder `launch`, and inside, create a new file `circle_path_launch.py`.

In this file, create a launch description that launches both nodes.
You will need to import the `Node` class from `launch_ros.actions` and the `LaunchDescription` class from `launch`.

Create the function `generate_launch_description` that returns a `LaunchDescription` object.
The launch description should contain two `Node` actions, one for each node.
Make sure that the two use the same `namespace` argument, so that they can find each other. Further, you will need to set the `pacakge` to `p_controller` and the `executable` to the node names.

The node for turtlesim will look like this:

```python
Node(
    package='turtlesim',
    namespace='test_namespace',
    executable='turtlesim_node',
    name='sim',
),
```

Once this file is saved, you should be able to launch it from the workspace root:

```terminal
cd ~/ros2_ws/src
ros2 launch p_controller/launch/circle_path_launch.py
```

You should see both nodes starting up, you should see the turtle moving in a circle. Note that turtlesim needs a little time to initialize, so the turtle will not start moving immediately.

Congratulations, you have successfully written your first ROS2 node!
