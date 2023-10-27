from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from ament_index_python import get_package_share_directory
#launch the new controller system

THRUSTER_SOLVER_NAME = "thruster_solver"
FEED_FORWARD_CONTROLLER_NAME = "FF_Controller"

def generate_launch_description():

    #get vehicle configuration directory
    vehicle_config_file = PathJoinSubstitution([
        get_package_share_directory("riptide_descriptions2"),
        "config", 
        LaunchConfiguration("robot_yaml")
    ])

    #declare overseer node
    overseerNode = Node(package="riptide_controllers2",
                        namespace=LaunchConfiguration("robot"),
                        executable="controller_overseer",
                        name="controller_overseer",
                        parameters=[{"vehicle_config": vehicle_config_file, 
                                     "robot": LaunchConfiguration("robot"),
                                     "thruster_solver_node_name": THRUSTER_SOLVER_NAME,
                                     "ff_controller_node_name": FEED_FORWARD_CONTROLLER_NAME,
                                    }])
    
    #declare thruster solver node
    thrusterSolverNode = Node(package="thrustersolver",
                        namespace=LaunchConfiguration("robot"),
                        executable="ThrusterSolver",
                        name=THRUSTER_SOLVER_NAME)
    
    ffControllerNode = Node(package="ff_controller",
                            namespace = LaunchConfiguration("robot"),
                            executable="FF_Controller",
                            name=FEED_FORWARD_CONTROLLER_NAME)
    
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="tempest", description="name of robot used for namespacing"),
        DeclareLaunchArgument("robot_yaml", default_value=[LaunchConfiguration("robot"), ".yaml"], description="robot config file name"),
        overseerNode,
        thrusterSolverNode,
        ffControllerNode
    ])