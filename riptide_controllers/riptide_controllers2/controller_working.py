#!/usr/bin/env python3

# controller node
#
# Input topics:
#   odometry/filtered: Current state of the vehicle
#   orientation: Puts the angular controller in position mode. Sets angular target to given orientation
#   angular_velocity: Puts the angular controller in velocity mode. Sets angular target to given body-frame angular velocity
#   disable_angular: Puts the angular controller in disabled mode.
#   position: Puts the linear controller in position mode. Sets linear target to given world-frame position
#   linear_velocity: Puts the linear controller in velocity mode. Sets linear target to given body-frame linear velocity
#   disable_linear: Puts the linear controller in disabled mode.
#   off: Turns off the controller. This will stop all output from the controller and thruster will stop
#
# Output topics:
#   net_force: The force the robot should exert on the world to achieve the given target
#   ~requested_accel: The acceleration requested from the controllers. Used for calibration
#
# This node contains 4 parts. The linear controller, the angular controller, the acceleration calculator, and the trajectory reader.
# The linear and angular controllers return an acceleration the robot should eperience to achieve that controller's target.
# The acceleration calculator takes that acceleration and computes how much force the robot needs to exert to achieve that acceleration.
# The trajectory reader will feed current states to the controllers to follow a trajectory.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data # can replace this with others

import numpy as np
import yaml

from riptide_controllers2.Controllers import ControlMode, LinearCascadedPController, AngularCascadedPController, AccelerationCalculator
from riptide_msgs2.msg import ControllerCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rcl_interfaces.msg import SetParametersResult

# assumes order is xyz
def vect3_from_np(np_vect):
    return Vector3(x=np_vect[0], y=np_vect[1], z=np_vect[2])

class ControllerNode(Node):
        
    def __init__(self):
        super().__init__('riptide_controllers2')
        
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if(config_path == ''):
            self.get_logger().fatal("vehicle config file param not set or empty, exiting")

        with open(config_path, 'r') as stream:
            config = yaml.safe_load(stream)
        
        self.linearController = LinearCascadedPController()
        self.angularController = AngularCascadedPController()
        self.accelerationCalculator = AccelerationCalculator(config)

        # configure the controllers
        # linear controller first
        linear = config['controller']['linear']
        self.linearController.velocityP = np.array(linear['p_gains']['velocity'])
        self.linearController.positionP = np.array(linear['p_gains']['position'])
        self.linearController.maxVelocity = np.array(linear['max']['velocity'])
        self.linearController.maxAccel = np.array(linear['max']['acceleration'])

        # angular controller second
        angular = config['controller']['angular']
        self.angularController.velocityP = np.array(angular['p_gains']['velocity'])
        self.angularController.positionP = np.array(angular['p_gains']['position'])
        self.angularController.maxVelocity = np.array(angular['max']['velocity'])
        self.angularController.maxAccel = np.array(angular['max']['acceleration'])

        self.lastTorque = None
        self.lastForce = None

        # setup publishers 
        self.steadyPub = self.create_publisher(Bool, "controller/steady", qos_profile_system_default)
        self.statePub = self.create_publisher(JointState, "controller/state", qos_profile_system_default)
        self.forcePub = self.create_publisher(Twist, "controller/body_force", qos_profile_system_default)

        # setup all subscribers
        self.subs = []

        # position information subscribers
        self.subs.append(self.create_subscription(Odometry, "odometry/filtered", self.updateState, qos_profile_system_default))

        # control input subscribers
        self.subs.append(self.create_subscription(ControllerCommand, "controller/linear", self.setLinear, qos_profile_system_default))
        self.subs.append(self.create_subscription(ControllerCommand, "controller/angular", self.setAngular, qos_profile_system_default))
        
        # state information
        self.subs.append(self.create_subscription(Bool, "state/kill", self.switch_cb, qos_profile_sensor_data))

        # declare the configuration data
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_position_p_x', linear['p_gains']['position'][0]),
                ('linear_position_p_y', linear['p_gains']['position'][1]),
                ('linear_position_p_z', linear['p_gains']['position'][2]),
                ('linear_velocity_p_x', linear['p_gains']['velocity'][0]),
                ('linear_velocity_p_y', linear['p_gains']['velocity'][1]),
                ('linear_velocity_p_z', linear['p_gains']['velocity'][2]),
                ('angular_position_p_x', angular['p_gains']['position'][0]),
                ('angular_position_p_y', angular['p_gains']['position'][1]),
                ('angular_position_p_z', angular['p_gains']['position'][2]),
                ('angular_velocity_p_x', angular['p_gains']['velocity'][0]),
                ('angular_velocity_p_y', angular['p_gains']['velocity'][1]),
                ('angular_velocity_p_z', angular['p_gains']['velocity'][2]),
                ('linear_damping_x', linear['damping']['linear'][0]),
                ('linear_damping_y', linear['damping']['linear'][1]),
                ('linear_damping_z', linear['damping']['linear'][2]),
                ('linear_damping_rot_x', angular['damping']['linear'][0]),
                ('linear_damping_rot_y', angular['damping']['linear'][1]),
                ('linear_damping_rot_z', angular['damping']['linear'][2]),
                ('quadratic_damping_x', linear['damping']['quadratic'][0]),
                ('quadratic_damping_y', linear['damping']['quadratic'][1]),
                ('quadratic_damping_z', linear['damping']['quadratic'][2]),
                ('quadratic_damping_rot_x', angular['damping']['quadratic'][0]),
                ('quadratic_damping_rot_y', angular['damping']['quadratic'][1]),
                ('quadratic_damping_rot_z', angular['damping']['quadratic'][2]),
                ('maximum_linear_velocity_x', linear['max']['velocity'][0]),
                ('maximum_linear_velocity_y', linear['max']['velocity'][1]),
                ('maximum_linear_velocity_z', linear['max']['velocity'][2]),
                ('maximum_linear_acceleration_x', linear['max']['acceleration'][0]),
                ('maximum_linear_acceleration_y', linear['max']['acceleration'][1]),
                ('maximum_linear_acceleration_z', linear['max']['acceleration'][2]),
                ('maximum_angular_velocity_x', angular['max']['velocity'][0]),
                ('maximum_angular_velocity_y', angular['max']['velocity'][1]),
                ('maximum_angular_velocity_z', angular['max']['velocity'][2]),
                ('maximum_angular_acceleration_x', angular['max']['acceleration'][0]),
                ('maximum_angular_acceleration_y', angular['max']['acceleration'][1]),
                ('maximum_angular_acceleration_z', angular['max']['acceleration'][2]),
                ('volume', config["volume"]),
                ('cob_x', config["cob"][0]),
                ('cob_y', config["cob"][1]),
                ('cob_z', config["cob"][2])
            ]) 

        # initialize reconfigure
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info(f"Riptide controller initalized with params {config}")

    def parameters_callback(self, params):
        success = True
        for param in params:
            self.get_logger().info(f"updating {param.name} to {param.value}")
            if param.name == "maximum_linear_velocity_x":
                self.linearController.maxVelocity[0] = param.value
            elif param.name == "maximum_linear_velocity_y":
                self.linearController.maxVelocity[1] = param.value
            elif param.name == "maximum_linear_velocity_z":
                self.linearController.maxVelocity[2] = param.value
            elif param.name == "maximum_linear_acceleration_x":
                self.linearController.maxAccel[0] = param.value
            elif param.name == "maximum_linear_acceleration_y":
                self.linearController.maxAccel[1] = param.value
            elif param.name == "maximum_linear_acceleration_z":
                self.linearController.maxAccel[2] = param.value
            elif param.name == "maximum_angular_velocity_x":
                self.angularController.maxVelocity[0] = param.value
            elif param.name == "maximum_angular_velocity_y":
                self.angularController.maxVelocity[1] = param.value
            elif param.name == "maximum_angular_velocity_z":
                self.angularController.maxVelocity[2] = param.value
            elif param.name == "maximum_angular_acceleration_x":
                self.angularController.maxAccel[0] = param.value
            elif param.name == "maximum_angular_acceleration_y":
                self.angularController.maxAccel[1] = param.value
            elif param.name == "maximum_angular_acceleration_z":
                self.angularController.maxAccel[2] = param.value
            elif param.name == "linear_position_p_x":
                self.linearController.positionP[0] = param.value
            elif param.name == "linear_position_p_y":
                self.linearController.positionP[1] = param.value
            elif param.name == "linear_position_p_z":
                self.linearController.positionP[2] = param.value
            elif param.name == "linear_velocity_p_x":
                self.linearController.velocityP[0] = param.value
            elif param.name == "linear_velocity_p_y":
                self.linearController.velocityP[1] = param.value
            elif param.name == "linear_velocity_p_z":
                self.linearController.velocityP[2] = param.value
            elif param.name == "angular_position_p_x":
                self.angularController.positionP[0] = param.value
            elif param.name == "angular_position_p_y":
                self.angularController.positionP[1] = param.value
            elif param.name == "angular_position_p_z":
                self.angularController.positionP[2] = param.value
            elif param.name == "angular_velocity_p_x":
                self.angularController.velocityP[0] = param.value
            elif param.name == "angular_velocity_p_y":
                self.angularController.velocityP[1] = param.value
            elif param.name == "angular_velocity_p_z":
                self.angularController.velocityP[2] = param.value
            elif param.name == "linear_damping_x":
                self.accelerationCalculator.linearDrag[0] = param.value
            elif param.name == "linear_damping_y":
                self.accelerationCalculator.linearDrag[1] = param.value
            elif param.name == "linear_damping_z":
                self.accelerationCalculator.linearDrag[2] = param.value
            elif param.name == "linear_damping_rot_x":
                self.accelerationCalculator.linearDrag[3] = param.value
            elif param.name == "linear_damping_rot_y":
                self.accelerationCalculator.linearDrag[4] = param.value
            elif param.name == "linear_damping_rot_z":
                self.accelerationCalculator.linearDrag[5] = param.value
            elif param.name == "quadratic_damping_x":
                self.accelerationCalculator.quadraticDrag[0] = param.value
            elif param.name == "quadratic_damping_y":
                self.accelerationCalculator.quadraticDrag[1] = param.value
            elif param.name == "quadratic_damping_z":
                self.accelerationCalculator.quadraticDrag[2] = param.value
            elif param.name == "quadratic_damping_rot_x":
                self.accelerationCalculator.quadraticDrag[3] = param.value
            elif param.name == "quadratic_damping_rot_y":
                self.accelerationCalculator.quadraticDrag[4] = param.value
            elif param.name == "quadratic_damping_rot_z":
                self.accelerationCalculator.quadraticDrag[5] = param.value
            elif param.name == "maximum_linear_velocity_x":
                self.linearController.maxVelocity[0] = param.value
            elif param.name == "maximum_linear_velocity_y":
                self.linearController.maxVelocity[1] = param.value
            elif param.name == "maximum_linear_velocity_z":
                self.linearController.maxVelocity[2] = param.value
            elif param.name == "maximum_linear_acceleration_x":
                self.linearController.maxAccel[0] = param.value
            elif param.name == "maximum_linear_acceleration_y":
                self.linearController.maxAccel[1] = param.value
            elif param.name == "maximum_linear_acceleration_z":
                self.linearController.maxAccel[2] = param.value
            elif param.name == "maximum_angular_velocity_x":
                self.angularController.maxVelocity[0] = param.value
            elif param.name == "maximum_angular_velocity_y":
                self.angularController.maxVelocity[1] = param.value
            elif param.name == "maximum_angular_velocity_z":
                self.angularController.maxVelocity[2] = param.value
            elif param.name == "maximum_angular_acceleration_x":
                self.angularController.maxAccel[0] = param.value
            elif param.name == "maximum_angular_acceleration_y":
                self.angularController.maxAccel[1] = param.value
            elif param.name == "maximum_angular_acceleration_z":
                self.angularController.maxAccel[2] = param.value
            elif param.name == "volume":
                self.accelerationCalculator.buoyancy = np.array([0, 0, param.value * self.accelerationCalculator.density * self.accelerationCalculator.gravity])
            elif param.name == "cob_x":
                self.accelerationCalculator.cob[0] = param.value
            elif param.name == "cob_y":
                self.accelerationCalculator.cob[1] = param.value
            elif param.name == "cob_z":
                self.accelerationCalculator.cob[2] = param.value
            else:
                success = False
                
        return SetParametersResult(successful=success)


    def updateState(self, odomMsg):
        totalState = JointState()
        isSteady = Bool()
        forceTwist = Twist()

        self.get_logger().debug('Calculating controller desired accelerations')
        linearDesiredState = self.linearController.update(odomMsg)
        angularDesiredState = self.angularController.update(odomMsg)

        totalState.name = [*linearDesiredState.name, *angularDesiredState.name]
        totalState.position = [*linearDesiredState.position, *angularDesiredState.position]
        totalState.velocity = [*linearDesiredState.velocity, *angularDesiredState.velocity]
        totalState.effort = [*linearDesiredState.effort, *angularDesiredState.effort]
        self.statePub.publish(totalState)

        isSteady.data = self.linearController.steady and self.angularController.steady

        self.get_logger().debug(f"linear control mode: {self.linearController.controlMode}, angular control mode: {self.angularController.controlMode}")

        if(self.linearController.controlMode != ControlMode.DISABLED or self.angularController.controlMode != ControlMode.DISABLED):
            netForce, netTorque = self.accelerationCalculator.accelToNetForce(odomMsg, np.array(linearDesiredState.effort), np.array(angularDesiredState.effort))
            forceTwist.linear = vect3_from_np(netForce)
            forceTwist.angular = vect3_from_np(netTorque)
            self.get_logger().debug('calculated forces with enabled controller')

        self.steadyPub.publish(isSteady)        
        self.forcePub.publish(forceTwist)

    def setLinear(self, msg : ControllerCommand):
        # self.get_logger().info(f'Setting linear controller to mode {msg.mode}')
        self.linearController.setTargetPosition(msg.setpoint_vect, ControlMode(msg.mode))

    def setAngular(self, msg : ControllerCommand):
        # self.get_logger().info(f'Setting angular controller to mode {msg.mode}')
        if(msg.mode == ControllerCommand.POSITION):
            self.angularController.setTargetPosition(msg.setpoint_quat, ControlMode(msg.mode))
        else:
            self.angularController.setTargetPosition(msg.setpoint_vect, ControlMode(msg.mode))

    def switch_cb(self, msg : Bool):
        if msg.data:
            if (self.angularController.controlMode != ControlMode.DISABLED or 
                self.linearController.controlMode != ControlMode.DISABLED):
                
                self.get_logger().warning('Controller output disabled from kill switch assert')
        
            self.angularController.setTargetPosition(Vector3(), ControlMode.DISABLED)
            self.linearController.setTargetPosition(Vector3(), ControlMode.DISABLED)
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
    