import os

import launch
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Read in the vehicle's namespace through the command line or use the default value one is not provided

    # declare the path to the robot's vehicle description file
    config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LC("robot_yaml")
    ])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "robot", 
            default_value="tempest",
            description="Name of the vehicle",
        ),

        launch.actions.DeclareLaunchArgument(
            "robot_yaml", 
            default_value=[LC("robot"), '.yaml'],
            description="Name of the vehicle",
        ),
        
        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(
                LC("robot"),
            ),

            # create the nodes    
            launch_ros.actions.Node(
                package='riptide_teleop2',
                executable='ps3_teleop',
                name='ps3_teleop',
                respawn=True,
                output='screen',
                
                parameters=[{"vehicle_config": config}]
            )
        ], scoped=True)
    ])