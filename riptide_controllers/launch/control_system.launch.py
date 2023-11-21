from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from ament_index_python import get_package_share_directory
#launch the new controller system

THRUSTER_SOLVER_NAME = "thruster_solver"
ACTIVE_CONTROLLER_NAME = "SMC"

def enabledNode(
    package: str,
    executable: str,
    name: str,
    parameters: list = []
) -> GroupAction:
    launcharg_name = name + "_enabled"
    return GroupAction([
        DeclareLaunchArgument(launcharg_name, default_value="true"),
        Node(
            package = package,
            executable = executable,
            name = name,
            parameters = parameters,
            condition = IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration(launcharg_name), "'"
                ])
            )
        )
    ])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="tempest", description="name of robot used for namespacing"),
        DeclareLaunchArgument("robot_yaml", default_value=[LaunchConfiguration("robot"), ".yaml"], description="robot config file name"),

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
                        "active_controller_node_name": ACTIVE_CONTROLLER_NAME
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
                name = ACTIVE_CONTROLLER_NAME
            ),

            Node(
                package="riptide_controllers2",
                executable="calibrate_drag.py",
                name="calibrate_drag",
                output="screen"
            )

        ], scoped=True)
    ])
