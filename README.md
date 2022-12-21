# riptide_control
Riptide Control serves as a package group for all onboard control packages for our AUV platform riptide. This repository includes the riptide_controllers package as well as the riptide_teleop package.

## riptide_controllers
Riptide Controllers supports controlling an over acutated vehicle by first computing a feed forward and feed back forces in the body frame. The 6dof problem is then solved via force minimization across the 8 thrusters on our over-actuated vehicles. 

## riptide_teleop
Riptide Teleop supports both the keyboard teleop twist node as well as the standard ros joy node as control interfaces for commanding the vehicle while underway. This operation mode generally requires the vehicle to be tethered to an operator computer topside.