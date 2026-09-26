"""Launch the MPC controller in place of complete_controller.

The MPC's model comes from riptide_mpc config/models (see model_files); the
simulator keeps its own plant, so the two can be made to differ on purpose.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def model_files(context):
    """Vehicle file + MPC model file.

    mpc_model:
      ""        riptide_mpc config/models/<robot>.yaml: our estimate of the real
                vehicle (the default, on hardware and in sim)
      sim       the simulator's own plant (its resolved snapshot when running
                under the sim bringup): a perfect-model baseline
      <name>    config/models/<name>.yaml, e.g. talos_sim
      <path>    any hydrodynamics-schema file
    """
    robot = LaunchConfiguration("robot").perform(context)
    choice = LaunchConfiguration("mpc_model").perform(context)
    vehicle = LaunchConfiguration("mpc_vehicle_config").perform(context)
    hydro = LaunchConfiguration("mpc_hydrodynamics_config").perform(context)
    resolved = context.launch_configurations.get("resolved_config", "")
    models = os.path.join(get_package_share_directory("riptide_mpc"), "config", "models")
    if not hydro:
        if choice == "sim":
            if resolved and Path(resolved, "hydrodynamics.yaml").is_file():
                hydro = str(Path(resolved, "hydrodynamics.yaml"))
                vehicle = vehicle or str(Path(resolved, "vehicle.yaml"))
            else:  # riptide_mpc's copy of the simulator plant
                hydro = os.path.join(models, f"{robot}_sim.yaml")
        elif choice and os.path.sep in choice:
            hydro = choice
        else:
            hydro = os.path.join(models, f"{choice or robot}.yaml")
    if not os.path.isfile(hydro):
        raise FileNotFoundError(f"MPC model file not found: {hydro}")
    vehicle = vehicle or os.path.join(get_package_share_directory("riptide_descriptions2"), "config", f"{robot}.yaml")
    return vehicle, hydro


def launch_mpc(context, *args, **kwargs):
    vehicle, hydro = model_files(context)
    print(f"MPC controller model: {vehicle} + {hydro}")
    overrides = {"vehicle_config": vehicle, "hydrodynamics_config": hydro}
    odom_topic = LaunchConfiguration("mpc_odom_topic").perform(context)
    state_source = LaunchConfiguration("mpc_state_source").perform(context)
    if odom_topic:
        overrides["odom_topic"] = odom_topic
        # A replacement odometry (e.g. ground truth) is meant to be used directly.
        state_source = state_source or "odometry"
    if state_source:
        overrides["state_source"] = state_source
    return [
        Node(
            package="riptide_mpc",
            executable="mpc_controller",
            name="mpc_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("mpc_config"),
                overrides,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument(
            "mpc_config",
            default_value=os.path.join(get_package_share_directory("riptide_mpc"), "config", "mpc.yaml")),
        DeclareLaunchArgument("mpc_model", default_value="",
                              description="MPC model: '' = config/models/<robot>.yaml (estimate), 'sim' = the "
                                          "simulator's plant, a name in config/models, or a path"),
        DeclareLaunchArgument("mpc_vehicle_config", default_value="",
                              description="Override the vehicle YAML used by the MPC model"),
        DeclareLaunchArgument("mpc_hydrodynamics_config", default_value="",
                              description="Override the hydrodynamics YAML used by the MPC model"),
        DeclareLaunchArgument("mpc_odom_topic", default_value="",
                              description="State feedback topic; e.g. simulator/ground_truth to bypass the EKF"),
        DeclareLaunchArgument("mpc_state_source", default_value="",
                              description="sensors (default) or odometry; mpc_odom_topic implies odometry"),
        OpaqueFunction(function=launch_mpc),
    ])
