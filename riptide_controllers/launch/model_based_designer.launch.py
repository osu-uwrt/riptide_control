# Launch the model-based gain designer.
#
# Identifies effective inertia (roll/pitch/yaw) + pitch stiffness in-sim, then computes overdamped
# PID gains (and optionally SMC lambda). Arm the vehicle in RViz first, with clearance.
#
#   ros2 launch riptide_controllers2 model_based_designer.launch.py robot:=talos
#   # more conservative / slower:  zeta:=1.4  wn_pid:="[2.0,2.0,1.5]"
#   # apply:=True (default) sets the params live on complete_controller (verified by readback);
#   # persist:=True (default) also patches the gain lines in the vehicle yaml so an overseer
#   # reload or stack restart doesn't revert the tune.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument("zeta", default_value="1.2",
                              description="damping ratio (>=1 overdamped, stability first)"),
        DeclareLaunchArgument("apply", default_value="True",
                              description="set designed gains live on the controller"),
        DeclareLaunchArgument("persist", default_value="True",
                              description="also patch the vehicle yaml so restarts keep the tune"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            Node(
                package="riptide_controllers2",
                executable="model_based_designer.py",
                name="model_based_designer",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "robot": LaunchConfiguration("robot"),
                    "zeta": ParameterValue(LaunchConfiguration("zeta"), value_type=float),
                    "apply": ParameterValue(LaunchConfiguration("apply"), value_type=bool),
                    "persist": ParameterValue(LaunchConfiguration("persist"), value_type=bool),
                }],
            ),
        ], scoped=True),
    ])
