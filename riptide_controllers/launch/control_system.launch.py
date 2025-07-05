from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration

import os

def get_complete_controller_launch(launch_prefix):
    return Node(
        prefix=[launch_prefix],
        package="complete_controller",
        executable="complete_controller",
        name="complete_controller",
        output="screen"
    )
    
def get_liltank_controller_launch(launch_prefix):
    return Node(
        prefix=[launch_prefix],
        package="liltank_controller",
        executable="liltank_controller",
        name="liltank_controller",
        output="screen"
    )

def get_launch_prefix(context):
    if LaunchConfiguration("core_reservation_enabled").perform(context) != "True":
        return ""
    
    #detect if we are running on the orin
    launch_prefix = ""
    if(os.path.exists("/home/ros/colcon_deploy")):
        print("I'm running on the orin! Isolating a core for controller use!")
        launch_prefix = "taskset -c 11"
    else:
        print("I'm running on a development laptop")
        
    return launch_prefix
    

def launch_control_system(context, *args, **kwargs):
    launch_prefix = get_launch_prefix(context)
    launches = [
        Node(
            prefix=[launch_prefix],
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
        )
    ]
        
    active_control_enabled = LaunchConfiguration("active_control_enabled").perform(context)   

    if active_control_enabled == "True":
        robot = LaunchConfiguration("robot").perform(context)
        if robot == "talos":
            launches.append(get_complete_controller_launch(get_launch_prefix(context)))
            return launches
        elif robot == "liltank":
            launches.append(get_liltank_controller_launch(get_launch_prefix(context)))
            return launches
        
    print("-----------------------------------------------------------------")
    print("Active control model either unknown or disabled, or robot name is unknown. Not launching.")
    print("-----------------------------------------------------------------")
    return launches

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="robot", default_value="tempest",
                              description="name of the robot to run"),
        
        DeclareLaunchArgument(name="robot_yaml", default_value=[LaunchConfiguration("robot"), ".yaml"],
                              description="Name of the robot yaml to use"),
        
        DeclareLaunchArgument(name="active_control_enabled", default_value="True",
                              description="Whether or not the active control model should be launched"),
        
        DeclareLaunchArgument(name="core_reservation_enabled", default_value="True",
                              description="Whether or not to reserve a core on Jetson computers"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            
            OpaqueFunction(function=launch_control_system)
        ], scoped=True)
    ])