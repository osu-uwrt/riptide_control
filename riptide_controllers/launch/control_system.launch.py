from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression, AndSubstitution, TextSubstitution
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from ament_index_python import get_package_share_directory
#launch the new controller system

THRUSTER_SOLVER_NAME = "thruster_solver"
DEFAULT_ACTIVE_CONTROLLER_NAME = "SMC"

def enabledNode(
    package: str,
    executable: str,
    name: str,
    enabled_by_default = True,
    parameters: list = []
) -> GroupAction:
    launcharg_name = name + "_enabled"
    
    actions = [
        DeclareLaunchArgument(launcharg_name, default_value="true")
    ]

    if enabled_by_default:
        node = Node(
            package = package,
            executable = executable,
            name = name,
            parameters = parameters,
            output="screen",
            condition = IfCondition(
                    PythonExpression(["'", LaunchConfiguration(launcharg_name), "'"])
            )
        )
        
        actions.append(node);    
    
    return GroupAction(actions)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="tempest", description="name of robot used for namespacing"),
        DeclareLaunchArgument("robot_yaml", default_value=[LaunchConfiguration("robot"), ".yaml"], description="robot config file name"),
        DeclareLaunchArgument("active_control_model", default_value=DEFAULT_ACTIVE_CONTROLLER_NAME, description="name of the simulink model to run"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),

            enabledNode(
                package = "riptide_controllers2",
                executable = "controller_overseer.py",
                name = "controller_overseer",
                parameters = [
                    {"vehicle_config": "",  # Leave empty to let the node discover it
                        "robot": LaunchConfiguration("robot"),
                        "thruster_solver_node_name": THRUSTER_SOLVER_NAME,
                        "active_controller_node_name": LaunchConfiguration("active_control_model")
                    }]
            ),

            enabledNode(
                package = "thruster_solver",
                executable = "thruster_solver",
                name = THRUSTER_SOLVER_NAME
            ),
            
            enabledNode(
                package = "smc",
                executable = "SMC",
                name = "SMC",
                enabled_by_default=False
            ),
            
            enabledNode(
                package = "pid",
                executable = "PID",
                name = "PID",
                enabled_by_default=True
            ),

            Node(
                package="riptide_controllers2",
                executable="calibrate_drag.py",
                name="calibrate_drag",
                output="screen"
            )

        ], scoped=True)
    ])
