# Exercise 2: Services

In this exercise, you will write a simple service that will allow you to change the radius of the cicle we drew in the previous exercise.

## Writing the service

We will modify the `circle_controller` node from the previous exercise to create a service that will allow us to change the radius of the circle.

Modify the `~/ros2_ws/src/p_controller/p_controller/circle_controller.py` file to add a service that will allow us to change the radius of the circle.
For this, you will have to create the service in the controller's constructor, and implement the service callback function:

```python
from p_interfaces.srv import ChangeRadius

class CircleController(Node):
    def __init__(self):
        # [...]
        self.srv = self.create_service(
            ChangeRadius, 'change_radius', self.change_radius_callback)

    def change_radius_callback(self, request, response):
        # [...]
        return response
```

If you try to run this, you will notice that the `ChangeRadius` service type is not known.
We will have to create this service type first.

Create a new package `p_interfaces` in your workspace:

```terminal
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake p_interfaces
```

This will create a new package with the name `p_interfaces` in your workspace.
Create the file `~/ros2_ws/src/p_interfaces/p_interfaces/srv/ChangeRadius.srv` with the following content:

```
int64 radius
---
bool radius_changed
geometry_msgs/Vector3 linear_velocity
```

This defines a service type called `ChangeRadius` that has a single field `radius` of type `int64`.
The return type of the service is a message with two fields: `radius_changed` of type `bool`, and `linear_velocity` of type `geometry_msgs/Vector3`.

Before building the `p_interfaces` package, we will have to add the dependencies in the `src/p_interfaces/CMakeLists.txt` file:

```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generatorsREQUIRED)
rosidl_generate_interfaces(\${PROJECT_NAME}
    "srv/ChangeRadius.srv"
    DEPENDENCIES geometry_msgs # Add packages that above msg and srv
                               # depend on, in this case geometry_msgs
)
```

We also need to add the dependency in `src/p_interfaces/package.xml`:

```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Now you can build the `p_interfaces` package:

```terminal
cd ~/ros2_ws
colcon build --packages-select p_interfaces
```

Don't forget to re-source your workspace:

```terminal
source install/setup.bash
```

Finally, we'll have to declare that `p_circle_controller` depends on `p_interfaces` in `src/p_controller/package.xml`:

```xml
<exec_depend>p_interfaces</exec_depend>
```

Since we modified the `package.xml` file, we'll have to rebuild the package:

```terminal
cd ~/ros2_ws
colcon build --packages-select --symlink-install p_controller
```

## Using the service

Now that we have the service implemented, we can use it to change the radius of the circle.

Launch the `circle_controller` node via the launch file as before:

```terminal
ros2 launch src/p_controller/p_controller/launch/circle_path_launch.py
```

(adjust the path depending on where you are in the terminal)

You can now list the available services:

```terminal
ros2 service list
[...]
/circle_controller/change_radius
```

And call it like such:

```terminal
ros2 service call /circle_controller/change_radius p_interfaces/ChangeRadius "{radius: 2}"
```

You should see the turtle change its radius.
