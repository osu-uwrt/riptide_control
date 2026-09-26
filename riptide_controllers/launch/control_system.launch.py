from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def get_complete_launch(launch_prefix):
    return Node(
        prefix=[launch_prefix],
        package="complete_controller",
        executable="complete_controller",
        name="complete_controller",
        output="screen"
    )

def get_launch_prefix():
    
    #detect if we are running on the orin
    launch_prefix = ""
    if(os.path.exists("/home/ros/colcon_deploy")):
        print("I'm running on the orin! Isolating a core for controller use!")
        launch_prefix = "taskset -c 11"
    else:
        print("I'm running on a development laptop")
        
    return launch_prefix
    

def launch_active_control(context, *args, **kwargs):
    active_control_enabled = LaunchConfiguration("active_control_enabled").perform(context)   
    active_control_model = LaunchConfiguration("active_control_model").perform(context)

    if active_control_enabled == "True" and active_control_model == "mpc":
        # Test MPC (riptide_mpc): same topics as complete_controller, Fossen-model prediction
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("riptide_mpc"), "launch", "mpc_controller.launch.py")),
            launch_arguments=[("robot", LaunchConfiguration("robot"))]
        )]

    if active_control_enabled == "True":
        return [get_complete_launch(get_launch_prefix())]
    
    print("-----------------------------------------------------------------")
    print("Active control model either unknown or disabled. Not launching.")
    print("-----------------------------------------------------------------")
    return []

def generate_launch_description():
    launch_prefix = get_launch_prefix()
    
    return LaunchDescription([
        DeclareLaunchArgument(name="robot", default_value="tempest",
                              description="name of the robot to run"),
        
        DeclareLaunchArgument(name="robot_yaml", default_value=[LaunchConfiguration("robot"), ".yaml"],
                              description="Name of the robot yaml to use"),
        
        DeclareLaunchArgument(name="active_control_enabled", default_value="True",
                              description="Whether or not the active control model should be launched"),

        DeclareLaunchArgument(name="active_control_model", default_value="hybrid",
                              description="Active controller: 'mpc' runs the riptide_mpc test controller, anything else complete_controller"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            
            Node(
                prefix=[launch_prefix],
                package="riptide_controllers2",
                executable="controller_overseer.py",
                name="controller_overseer",
                parameters = [
                    {
                        "vehicle_config": "",  # Leave empty to let the node discover it
                        "robot": LaunchConfiguration("robot"),
                        "active_control_model": LaunchConfiguration("active_control_model"),
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
