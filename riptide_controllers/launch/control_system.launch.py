from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

DEFAULT_ACTIVE_CONTROL_MODEL = "SMC"
THRUSTER_SOLVER_NAME = "thruster_solver"

def get_smc_launch():
    return Node(
        package="smc",
        executable="SMC",
        name="SMC",
        output="screen"
    )


def get_pid_launch():
    return Node(
        package="pid",
        executable="PID",
        name="PID",
        output="screen"
    )


def launch_active_control(context, *args, **kwargs):
    active_control_enabled = LaunchConfiguration("active_control_enabled").perform(context)
    active_control_model = LaunchConfiguration("active_control_model").perform(context)
    
    if active_control_enabled == "True":
        if active_control_model == "SMC":
            return [get_smc_launch()]
        elif active_control_model == "PID":
            return [get_pid_launch()]
    
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
        
        DeclareLaunchArgument(name="thruster_solver_enabled", default_value="True",
                              description="Whether or not the thruster solver should be launched"),
        
        DeclareLaunchArgument(name="active_control_enabled", default_value="True",
                              description="Whether or not the active control model should be launched"),

        DeclareLaunchArgument(name="active_control_model", default_value=DEFAULT_ACTIVE_CONTROL_MODEL,
                              description="Name of the active control model to use"),
        
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
                        "thruster_solver_node_name": THRUSTER_SOLVER_NAME,
                        "active_controller_node_name": LaunchConfiguration("active_control_model")
                    }
                ],
                output="screen"
            ),
            
            Node(
                package="riptide_controllers2",
                executable="calibrate_drag.py",
                name="calibrate_drag",
                output="screen"
            ),
            
            Node(
                package="thruster_solver",
                executable="thruster_solver",
                name=THRUSTER_SOLVER_NAME,
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("thruster_solver_enabled"), "'"])
                )
            ),
            
            OpaqueFunction(function=launch_active_control)
        ], scoped=True)
    ])
