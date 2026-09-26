# Launch the Optuna FF auto-tuner under the robot namespace.
#
# Requires optuna in the environment:  pip install optuna
#
# Example:
#   ros2 launch riptide_controllers2 ff_optuna_tuner.launch.py robot:=talos n_trials:=20
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
        DeclareLaunchArgument("n_trials", default_value="200",
                              description="number of Optuna trials"),
        DeclareLaunchArgument("trial_duration", default_value="5.0",
                              description="seconds of feedforward per trial"),
        DeclareLaunchArgument("write_config", default_value="False",
                              description="patch base_wrench into the vehicle yaml when done"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),

            Node(
                package="riptide_controllers2",
                executable="ff_optuna_tuner.py",
                name="ff_optuna_tuner",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "robot": LaunchConfiguration("robot"),
                    "n_trials": ParameterValue(LaunchConfiguration("n_trials"), value_type=int),
                    "trial_duration": ParameterValue(
                        LaunchConfiguration("trial_duration"), value_type=float),
                    "write_config": ParameterValue(
                        LaunchConfiguration("write_config"), value_type=bool),
                }],
            ),
        ], scoped=True),
    ])
