# Launch the Optuna controller-gain tuner under the robot namespace.
#
# Requires optuna:  pip install optuna
#
# Examples:
#   # tune yaw in BOTH PID and SMC (default):
#   ros2 launch riptide_controllers2 controller_optuna_tuner.launch.py robot:=talos
#   # tune only SMC on pitch+yaw:
#   ros2 launch riptide_controllers2 controller_optuna_tuner.launch.py robot:=talos \
#        pid_axes:="[]" smc_axes:="[4,5]"
#
# IMPORTANT: arm the vehicle in RViz and stage it with room for the step maneuvers.
# Recovery always uses the ORIGINAL gains, so a bad candidate is penalized, not catastrophic.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument("pid_axes", default_value="3,4,5",
                              description="axis indices [x,y,z,roll,pitch,yaw]=0..5 tuned in PID (default: roll,pitch,yaw)"),
        DeclareLaunchArgument("smc_axes", default_value="0,1,2",
                              description="axis indices tuned in SMC (default: x,y,z)"),
        DeclareLaunchArgument("n_trials", default_value="120"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            Node(
                package="riptide_controllers2",
                executable="controller_optuna_tuner.py",
                name="controller_optuna_tuner",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "robot": LaunchConfiguration("robot"),
                    "pid_axes": ParameterValue(LaunchConfiguration("pid_axes"), value_type=str),
                    "smc_axes": ParameterValue(LaunchConfiguration("smc_axes"), value_type=str),
                    "n_trials": ParameterValue(LaunchConfiguration("n_trials"), value_type=int),
                }],
            ),
        ], scoped=True),
    ])
