import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Models and namespaces for each TurtleBot
    turtlebot_configurations = [
        {'model': 'waffle', 'namespace': 'waffle'},
        {'model': 'burger', 'namespace': 'burger'}
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # List to hold spawned nodes
    nodes = []

    for config in turtlebot_configurations:
        urdf_file_name = 'turtlebot3_' + config['model'] + '.urdf'
        urdf_path = os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            urdf_file_name)

        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()

        node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=config['namespace'] + '_robot_state_publisher',
            output='screen',
            namespace=config['namespace'],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        )

        nodes.append(node)

    ld = LaunchDescription()

    # Add all nodes to LaunchDescription
    for node in nodes:
        ld.add_action(node)

    return ld
