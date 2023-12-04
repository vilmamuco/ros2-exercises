import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

   


def generate_launch_description():
    pkg_bluerov_test = get_package_share_directory('exercise3')

    fcu_url_arg ='udp://0.0.0.0:14550@:14549'

    gcs_url_arg = 'udp://@'


    mavros_path = get_package_share_directory('mavros')
    mavros_launch_path = os.path.join(
        mavros_path, 'launch', 'apm.launch')
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            'fcu_url': fcu_url_arg,
            'gcs_url': gcs_url_arg,
            'tgt_system': '1',
            'tgt_component': "1",
            }.items()
        )

    agent_node = Node(
        package='exercise3',
        executable='bluerov',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,
        #  gazebo ./worlds/underwater.world
        # ./sim_bluerov/ardupilot/Tools/autotest/sim_vehicle.py 
        # -f gazebo-bluerov2 -L RATBeach --console -v ArduSub
        # Node(
        #     package='ping360_sonar',
        #     executable='ping360_node',
        #     parameters=[{
        #         'angle_sector': 90,
        #         'connection_type': 'udp',
        #         'udp_address': '192.168.2.2',
        #         'udp_port': 9092,
        #         'fallback_emulated': False,
        #         'publish_echo': True,
        #         'publish_scan': True,
        #         'publish_image': True,
        #     }],
        # ),
        agent_node,
        ExecuteProcess(
            cmd=[
                '/home/docker/ardupilot/Tools/autotest/sim_vehicle.py', '-v', 'ArduSub',
                '-L', 'RATBeach', '-f', 'gazebo-bluerov2', '--console'],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=[
        #         'gazebo', '/home/docker/bluerov_ros_playground/worlds/underwater.world'],
        #     output='screen'
        # ),
    ])
