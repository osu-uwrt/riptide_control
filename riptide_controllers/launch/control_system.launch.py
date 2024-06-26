from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration

def get_complete_launch():
    return Node(
        package="complete_controller",
        executable="complete_controller",
        name="complete_controller",
        output="screen"
    )


def launch_active_control(context, *args, **kwargs):
    active_control_enabled = LaunchConfiguration("active_control_enabled").perform(context)    
    if active_control_enabled == "True":
        return [get_complete_launch()]
    
    print("-----------------------------------------------------------------")
    print("Active control model either unknown or disabled. Not launching.")
    print("-----------------------------------------------------------------")
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="robot", default_value="tempest",
                              description="name of the robot to run"),
        
        DeclareLaunchArgument(name="robot_yaml", default_value=[LaunchConfiguration("robot"), ".yaml"],
                              description="Name of the robot yaml to use"),
        
        DeclareLaunchArgument(name="active_control_enabled", default_value="True",
                              description="Whether or not the active control model should be launched"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            
            Node(
                package="riptide_controllers2",
                executable="controller_overseer.py",
                name="controller_overseer",
                parameters = [
                    {
                        "vehicle_config": "",  # Leave empty to let the node discover it
                        "robot": LaunchConfiguration("robot"),
                    }
                ],
                output="screen"
            ),
            
            Node(
                package="riptide_controllers2",
                executable="calibrate_drag.py",
                name="calibrate_drag",
                output="screen",
            ),

            OpaqueFunction(function=launch_active_control)
        ], scoped=True)
    ])
