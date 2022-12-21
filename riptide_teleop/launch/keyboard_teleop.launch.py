import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    # Read in the vehicle's namespace through the command line or use the default value one is not provided

    # declare the path to the robot's vehicle description file
    config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LaunchConfiguration("robot_yaml")
    ])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "robot", 
            default_value="tempest",
            description="Name of the vehicle",
        ),

        launch.actions.DeclareLaunchArgument(
            "robot_yaml", 
            default_value=[LaunchConfiguration("robot"), '.yaml'],
            description="Name of the vehicle",
        ),

        # create the nodes    
        launch_ros.actions.Node(
            package='riptide_teleop2',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            respawn=False,
            output='screen',
            
            parameters=[{"vehicle_config": config}]
        )
    ])