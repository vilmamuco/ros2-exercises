# Exercise 3: Actions

An action combines the request-response mechanism of services with the publish-subscribe mechanism of topics.

In this exercise, you will write a simple action that will allow you to tell the turtle to go to a specified coordinate.
In doing so, it will give regular updates about its progress.

## Writing the action

```terminal
cd ~/ros2_ws/src
ros2 pkg create --license Apache-2.0 p_action_interfaces
cd p_action_interfaces
mkdir action
```

Inside the `action` folder, we'll create the file `GoToPoint.action` with the following content:

```
# Request
float32 x
float32 y
---
# Result
bool success
---
# Feedback
turtlesim/Posepose
```

As with the service exercise, we'll modify the `p_action_interfaces/CMakeLists.txt` file to add the dependencies:

```cmake
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    "action/GoToPoint.action"
    DEPENDENCIES turtlesim
)
ament_package()
```

And we'll also need to declare these dependencies in the `package.xml` file:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<exec_depend>turtlesim</exec_depend>
```

Now we can build the package and source the workspace:

```terminal
cd ~/ros2_ws
colcon build --packages-select p_action_interfaces
source install/setup.bash
```

Verify that the action interface was generated correctly:

```terminal
ros2 interface show p_action_interfaces/action/GoToPoint
```

To implement the action server node, create a new package `p_action_server` with the `ros pkg create` command as before:

```terminal
cd ~/ros2_ws/src

ros2 pkg create \
    --build-type ament_python \
    --node-name p_action_server p_action_example \
    --license Apache-2.0
```

Modify the package's `p_action_server` node to implement the action server.
You will need to...

- Import the `p_action_interfaces/action/GoToPoint` action type
  Don't forget to add it to `package.xml` and rebuild the package!
- Create a class `GoToGoal` that inherits from `Node`
- Create a publisher for the velocities
- Create a subscriber for the pose of the turtle
- Create the actual action server, with a callback function that is called when a new goal is _accepted_
- Create the pose subscrption callback function, publishing an appropriate angular and lateral velocity as a function of the turtle's current vs desired pose
- `spin()` the node using a `MultiThreadedExecutor` to make sure all these things can happen concurrently.

See the slides for details or if you get stuck.

## Writing the action client

Once you have the action server working, create a new python file `p_action_client.py` inside the `p_action_example` package.

It will contain the action client, which you will have to implement, and we'll attempt to make the turtle draw a square by sending it four goals, one after the other.

In outline, this node will have to...

- Import the `p_action_interfaces/action/GoToPoint` action type
- Create an `ActionClient` for the `GoToPoint` action type
- Create a list of goal coordinates like `[(0,0), (0,1), (1,1), (1,0)]`
- Call the action with the first goal
- Every time that a new feedback message is received, print it to the screen
- Once the goal is finished, call the action with the next goal
- Once all goals are finished, exit the node

Again, see the slides for more details.

To run both nodes, create a launch file `p_action_example/launch/action_example_launch.py` that includes _both_ nodes, plus the turtlesim node.
Alternatively, launch three terminals in your docker container, and launch each node individually.
