"""Pool identification session (see src/pool_identify_node.cpp).

The MPC must already be running (active_control_model:=mpc). Arm the vehicle at
depth, with the kill switch in hand, in open water around the start point:
  surge/sway lanes of `lane_length` (+x / +y body from the start heading),
  `heave_span` below the start depth, and a yaw sweep in place.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def overrides(context, *args, **kwargs):
    params = {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"}
    model = LaunchConfiguration("model").perform(context)
    if model:
        if os.path.sep not in model:
            model = os.path.join(get_package_share_directory("riptide_mpc"), "config", "models", model + ".yaml")
        if not os.path.isfile(model):
            raise FileNotFoundError(f"prior model not found: {model}")
        params["hydrodynamics_config"] = model
    output_dir = LaunchConfiguration("output_dir").perform(context)
    if output_dir:
        params["output_dir"] = os.path.expanduser(output_dir)
    return [Node(
        package="riptide_mpc",
        executable="pool_identify",
        name="pool_identify",
        namespace=LaunchConfiguration("robot"),
        output="screen",
        parameters=[LaunchConfiguration("config"), params],
    )]


def generate_launch_description():
    share = get_package_share_directory("riptide_mpc")
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "identification.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("model", default_value="",
                              description="Prior model: a name in riptide_mpc config/models or a path "
                                          "(default: config/models/<robot>.yaml). Use the model the MPC is running."),
        DeclareLaunchArgument("output_dir", default_value="",
                              description="Session folder root (default ~/osu-uwrt/mpc_identification)"),
        OpaqueFunction(function=overrides),
    ])
