import launch
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    # Read in the vehicle's namespace through the command line or use the default value one is not provided
    robot = LaunchConfiguration("robot")

    # declare the path to the robot's vehicle description file
    config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LaunchConfiguration("robot_yaml")
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="tempest",
            description="Name of the vehicle",
        ),
        DeclareLaunchArgument('robot_yaml', default_value=[
                              LaunchConfiguration("robot"), '.yaml']),

        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(
                LC("robot")
            ),

            launch_ros.actions.Node(
                package="riptide_controllers2",
                executable="calibrate_thruster",
                name="calibrate_thruster",
                output="screen",
                parameters=[
                    {"vehicle_config": config},
                    {"robot": robot},
                ]
            ),
        ], scoped=True)
    ])
