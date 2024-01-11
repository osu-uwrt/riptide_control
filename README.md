# riptide_control
Riptide Control serves as a package group for all onboard control packages for our AUV platform riptide. This repository includes the riptide_controllers package as well as the riptide_teleop package.

|            |              |
|------------|--------------|
| OS Distro  | Ubuntu 22.04 |
| ROS Distro | ROS2 Humble  |

## riptide_controllers
Riptide Controllers supports controlling an over acutated vehicle by first computing a feed forward and feed back forces in the body frame. The 6dof problem is then solved via force minimization across the 8 thrusters on our over-actuated vehicles. 

## riptide_teleop
Riptide Teleop supports both the keyboard teleop twist node as well as the standard ros joy node as control interfaces for commanding the vehicle while underway. This operation mode generally requires the vehicle to be tethered to an operator computer topside.

## Launch 
Run PID in loop:

ros2 launch riptide_controllers2 control_system.launch.py robot:=talos active_control_model:=PID PID_enabled:=false

Run PID built:

ros2 launch riptide_controllers2 control_system.launch.py robot:=talos active_control_model:=PID PID_enabled:=true
