from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

DEFAULT_ACTIVE_CONTROL_MODEL = "hybrid"
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


def get_complete_launch():
    return Node(
        package="complete_controller",
        executable="complete_controller",
        name="complete_controller",
        output="screen"
    )
    

def get_thruster_solver_launch():
    return Node(
        package="thruster_solver",
        executable="thruster_solver",
        name=THRUSTER_SOLVER_NAME,
        output="screen"
    )


def launch_active_control(context, *args, **kwargs):
    active_control_enabled = LaunchConfiguration("active_control_enabled").perform(context)
    active_control_model = LaunchConfiguration("active_control_model").perform(context)
    
    print(f"Active control model set to: {active_control_model}")
    
    if active_control_enabled == "True":
        if active_control_model == "SMC":
            return [get_smc_launch()]
        elif active_control_model == "PID":
            return [get_pid_launch()]
        elif active_control_model == "hybrid":
            return [get_smc_launch(), get_pid_launch()]
        elif active_control_model == "complete":
            return [get_complete_launch()]
    
    print("-----------------------------------------------------------------")
    print("Active control model either unknown or disabled. Not launching.")
    print("-----------------------------------------------------------------")
    return []


def launch_thruster_solver(context, *args, **kwargs):
    active_control_model = LaunchConfiguration("active_control_model").perform(context)
    thruster_solver_enabled = LaunchConfiguration("thruster_solver_enabled").perform(context)
    if active_control_model != "complete" and thruster_solver_enabled == "True":
        return [get_thruster_solver_launch()]
    
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

            OpaqueFunction(function=launch_thruster_solver),
            OpaqueFunction(function=launch_active_control)
        ], scoped=True)
    ])
