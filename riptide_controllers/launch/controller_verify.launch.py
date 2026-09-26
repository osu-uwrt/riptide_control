# Launch the controller verification lap.
#
# Holds at home (station-keeping RMS per axis), then runs small step tests on x, y, z, yaw
# (never roll/pitch) and reports rise/overshoot/settle/oscillation. Writes a YAML report.
# Run it before and after a tuning pass with different label:= values and compare.
#
#   ros2 launch riptide_controllers2 controller_verify.launch.py robot:=talos label:=baseline
#
# IMPORTANT: arm the vehicle in RViz and stage it with clearance for the step maneuvers.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument("label", default_value="verify",
                              description="tag for the report file, e.g. baseline / tuned"),
        DeclareLaunchArgument("handoff", default_value="False",
                              description="leave the vehicle holding at home instead of disarming"),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot")),
            Node(
                package="riptide_controllers2",
                executable="controller_verify.py",
                name="controller_verify",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "robot": LaunchConfiguration("robot"),
                    "label": ParameterValue(LaunchConfiguration("label"), value_type=str),
                    "handoff": ParameterValue(LaunchConfiguration("handoff"), value_type=bool),
                }],
            ),
        ], scoped=True),
    ])
