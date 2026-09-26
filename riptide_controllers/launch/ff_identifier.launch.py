# Launch the closed-loop FF identifier under the robot namespace.
#
# Example:
#   ros2 launch riptide_controllers2 ff_identifier.launch.py robot:=talos
#
# IMPORTANT: arm the vehicle in RViz (ControlPanel) and stage it with clearance
# greater than abort_radius before starting. Validate in the simulator first.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos",
                              description="robot namespace / vehicle name"),
        DeclareLaunchArgument("n_iterations", default_value="8",
                              description="number of measure/update iterations"),
        DeclareLaunchArgument("measure_window", default_value="1.5",
                              description="seconds of feedforward per measurement"),
        DeclareLaunchArgument("write_config", default_value="False",
                              description="patch base_wrench into the vehicle yaml when done"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            Node(
                package="riptide_controllers2",
                executable="ff_identifier.py",
                name="ff_identifier",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "robot": LaunchConfiguration("robot"),
                    "n_iterations": ParameterValue(
                        LaunchConfiguration("n_iterations"), value_type=int),
                    "measure_window": ParameterValue(
                        LaunchConfiguration("measure_window"), value_type=float),
                    "write_config": ParameterValue(
                        LaunchConfiguration("write_config"), value_type=bool),
                }],
            ),
        ], scoped=True),
    ])
