from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration as LC
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="tempest", description="Name of the robot"),
        
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen"),
        
        Node(
            package="riptide_controllers2",
            executable="talos_teleop.py",
            name="talos_teleop_node",
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", LC("robot"), "' == 'talos'"])
            )
        ),
        
        Node(
            package="riptide_controllers2",
            executable="liltank_teleop.py",
            name="liltank_teleop_node",
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", LC("robot"), "' == 'liltank'"])
            )
        )
    ])