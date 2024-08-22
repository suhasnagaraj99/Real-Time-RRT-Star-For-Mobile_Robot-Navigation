
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    turtlebot3_project5_dir = get_package_share_directory('turtlebot3_project5')
    gazebo_models_dir = os.path.join(turtlebot3_project5_dir, 'models')

    # Launch configuration variables specific to simulation
    turtlebot_count = 2  # Number of TurtleBots to spawn
    turtlebot_models = ['waffle', 'burger']  # Models for each TurtleBot
    x_poses = ['0.0', '1.0']  # X positions for each TurtleBot
    y_poses = ['0.0', '-0.8']  # Y positions for each TurtleBot
    yaw_angles = ['0.0', '1.5708']  # Yaw angles in radians for each TurtleBot (0, 90 degrees)

    # List to hold spawned entities
    spawn_entity_cmds = []

    # Iterate over each TurtleBot
    for i in range(turtlebot_count):
        model_folder = 'turtlebot3_' + turtlebot_models[i]
        urdf_path = os.path.join(gazebo_models_dir, model_folder, 'model.sdf')

        # Declare the launch arguments
        x_pose = LaunchConfiguration('x_pose_' + str(i), default=x_poses[i])
        y_pose = LaunchConfiguration('y_pose_' + str(i), default=y_poses[i])
        yaw_angle = LaunchConfiguration('yaw_angle_' + str(i), default=yaw_angles[i])

        declare_x_position_cmd = DeclareLaunchArgument(
            'x_pose_' + str(i), default_value=x_poses[i],
            description='X position of TurtleBot ' + str(i + 1))

        declare_y_position_cmd = DeclareLaunchArgument(
            'y_pose_' + str(i), default_value=y_poses[i],
            description='Y position of TurtleBot ' + str(i + 1))

        declare_yaw_angle_cmd = DeclareLaunchArgument(
            'yaw_angle_' + str(i), default_value=yaw_angles[i],
            description='Yaw angle of TurtleBot ' + str(i + 1))

        # Create the node to spawn the entity
        spawn_entity_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot' + str(i + 1),
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-Y', yaw_angle,
                '-robot_namespace', turtlebot_models[i],
            ],
            output='screen'
        )

        # Add actions to LaunchDescription
        spawn_entity_cmds.append(declare_x_position_cmd)
        spawn_entity_cmds.append(declare_y_position_cmd)
        spawn_entity_cmds.append(declare_yaw_angle_cmd)
        spawn_entity_cmds.append(spawn_entity_cmd)

    ld = LaunchDescription()

    # Add all actions to LaunchDescription
    for cmd in spawn_entity_cmds:
        ld.add_action(cmd)

    return ld

