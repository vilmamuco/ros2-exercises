from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='test_namespace',
            executable='turtlesim_node',
            name='sim',
            remappings=[
                ('/test_namespace/turtle2/cmd_vel',
                 '/test_namespace/turtle1/cmd_vel')
            ],
            parameters=[
                {"background_b": 200},
                {"background_g": 100},
                {"background_r": 150}
            ]
        ),
        Node(
            package='p_controller',
            namespace='test_namespace',
            executable='circle_controller',
            name='controller',
            parameters=[
                {"angular_z_velocity": 0.5}
            ]
        )
    ])
