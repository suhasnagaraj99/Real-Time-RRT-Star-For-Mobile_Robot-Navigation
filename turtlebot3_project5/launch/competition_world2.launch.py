import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_project5'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Define launch arguments for number of TurtleBots, models, and initial poses
    turtlebot_count = LaunchConfiguration('turtlebot_count', default='1')
    turtlebot_models = LaunchConfiguration('turtlebot_models', default='waffle')
    x_poses = LaunchConfiguration('x_poses', default='0.0')
    y_poses = LaunchConfiguration('y_poses', default='0.0')

    # Load world file
    world = os.path.join(
        get_package_share_directory('turtlebot3_project5'),
        'worlds',
        'final_world.world'
    )

    # Include Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Include Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Include robot state publisher launch
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher2.launch.py')
        )
    )

    # Include spawn turtlebot launch
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot32.launch.py')
        ),
        # Pass launch arguments for number of TurtleBots, models, and initial poses
        launch_arguments={
            'turtlebot_count': turtlebot_count,
            'turtlebot_models': turtlebot_models,
            'x_poses': x_poses,
            'y_poses': y_poses
        }.items()
    )

    # Declare additional arguments
    additional_args = [
        DeclareLaunchArgument('turtlebot_count', default_value='1',
                              description='Number of TurtleBots to spawn'),
        DeclareLaunchArgument('turtlebot_models', default_value='waffle',
                              description='Models for TurtleBots to spawn'),
        DeclareLaunchArgument('x_poses', default_value='0.0',
                              description='Initial X poses for TurtleBots'),
        DeclareLaunchArgument('y_poses', default_value='0.0',
                              description='Initial Y poses for TurtleBots')
    ]

    ld = LaunchDescription()

    # Add additional launch arguments
    for arg in additional_args:
        ld.add_action(arg)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
