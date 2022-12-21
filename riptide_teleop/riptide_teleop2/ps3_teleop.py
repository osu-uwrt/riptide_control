#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Quaternion
from transforms3d.euler import quat2euler, euler2quat
from transforms3d.quaternions import qmult
from riptide_msgs2.msg import ControllerCommand, KillSwitchReport
import numpy as np
import math
import yaml
import sys

AXES_STICK_LEFT_LR = 0 # Fully Leftwards = +1, Mid = 0, Fully Rightwards = -1
AXES_STICK_LEFT_UD = 1 # Fully Upwards = +1, Mid = 0, Fully Downwards = -1
AXES_STICK_RIGHT_LR = 3 # Fully Leftwards = +1, Mid = 0, Fully Rightwards = -1
AXES_STICK_RIGHT_UD = 4 # Fully Upwards = +1, Mid = 0, Fully Downwards = -1
AXES_REAR_L2 = 2 # Released = +1, Mid = 0, Fully Pressed = -1
AXES_REAR_R2 = 5 # Released = +1, Mid = 0, Fully Pressed = -1

#Joy Buttons
BUTTON_SHAPE_X = 0
BUTTON_SHAPE_CIRCLE = 1
BUTTON_SHAPE_TRIANGLE = 2
BUTTON_SHAPE_SQUARE = 3
BUTTON_REAR_L1 = 4
BUTTON_REAR_R1 = 5
BUTTON_REAR_L2 = 6
BUTTON_REAR_R2 = 7
BUTTON_SELECT = 8
BUTTON_START = 9
#BUTTON_PAIRING = 10
BUTTON_STICK_LEFT = 10
BUTTON_STICK_RIGHT = 11

BUTTON_CROSS_UP = 13
BUTTON_CROSS_DOWN = 14
BUTTON_CROSS_LEFT = 15
BUTTON_CROSS_RIGHT = 16

#Axes
# AXES_CROSS_LEFT_RIGHT = 4
# AXES_CROSS_UP_DOWN = 5

def msgToNumpy(msg):
    if hasattr(msg, "w"):
        return np.array([msg.w, msg.x, msg.y, msg.z])
    return np.array([msg.x, msg.y, msg.z])

# Joystick curve to apply. This is a x^2 that keeps polarity
def curve(val):
    return val * abs(val)

class PS3Teleop(Node):

    def __init__(self):
        super().__init__('ps3_teleop')

        self.declare_parameter('vehicle_config', '/config/puddles.yaml')
        self._vehicle_config_path = self.get_parameter("vehicle_config").value
        with open(self._vehicle_config_path, 'r') as stream:
            config = yaml.safe_load(stream)
            self.max_linear_velocity = config['controller']['linear']['max']['velocity']
            self.max_angular_velocity = config['controller']['angular']['max']['velocity']

        self.START_DEPTH = -1
        self.last_odom_msg = self.get_clock().now()
        self.last_linear_velocity = np.zeros(3)
        self.desired_orientation = np.array([1, 0, 0, 0])
        self.kill_msg = KillSwitchReport()
        self.ang_vel = np.zeros(3)
        self.enabled = False
        self.odom = None

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_cb,  qos_profile_system_default)
        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, qos_profile_system_default)

        self.software_kill_pub = self.create_publisher(KillSwitchReport, "control/software_kill", qos_profile_sensor_data)
        self.lin_pub = self.create_publisher(ControllerCommand, "controller/linear", qos_profile_system_default)
        self.ang_pub = self.create_publisher(ControllerCommand, "controller/angular", qos_profile_system_default)
      
    def joy_cb(self, msg):
        # Kill button
        #self.get_logger().info('buttons: {}'.format(msg.buttons))
        if msg.buttons[BUTTON_SHAPE_X]:
            self.enabled = False
            self.kill_msg.switch_asserting_kill = True
            return

        # Pause button
        elif self.enabled and msg.buttons[BUTTON_SHAPE_SQUARE]:
            self.enabled = False
            self.kill_msg.switch_asserting_kill = False
            self.last_linear_velocity = np.zeros(3)
            return

        # While controller is enabled
        if self.enabled:
            self.kill_msg.switch_asserting_kill = False

            # Build linear velocity
            linear_velocity = Vector3()
            linear_velocity.x = curve(msg.axes[AXES_STICK_RIGHT_UD]) * self.max_linear_velocity[0]
            linear_velocity.y = curve(msg.axes[AXES_STICK_LEFT_LR]) * self.max_linear_velocity[1]
            linear_velocity.z = curve(msg.axes[AXES_STICK_LEFT_UD]) * self.max_linear_velocity[2]

            # Build angular velocity
            # 0.9 is to allow the robot to catch up to the moving target
            self.ang_vel = np.zeros(3)
            self.ang_vel[2] = curve(msg.axes[AXES_STICK_RIGHT_LR]) * self.max_angular_velocity[2] * 0.9
            if msg.buttons[BUTTON_CROSS_UP]:
                self.ang_vel[1] = self.max_angular_velocity[1] * 0.9
            if msg.buttons[BUTTON_CROSS_DOWN]:
                self.ang_vel[1] = -self.max_angular_velocity[1] * 0.9
            if msg.buttons[BUTTON_CROSS_LEFT]:
                self.ang_vel[0] = -self.max_angular_velocity[0] * 0.9
            if msg.buttons[BUTTON_CROSS_RIGHT]:
                self.ang_vel[0] = self.max_angular_velocity[0] * 0.9


            # Publish linear velocity if the joystick has been touched
            if not np.array_equal(self.last_linear_velocity, msgToNumpy(linear_velocity)):
                lin_msg = ControllerCommand()
                lin_msg.setpoint_vect = linear_velocity
                lin_msg.mode = ControllerCommand.VELOCITY
                self.lin_pub.publish(lin_msg)
                self.last_linear_velocity = msgToNumpy(linear_velocity)

            # Zero roll and pitch
            if msg.buttons[BUTTON_SHAPE_CIRCLE]:
                r, p, y = quat2euler(self.desired_orientation, 'sxyz')
                r, p = 0, 0
                self.desired_orientation = euler2quat(r, p, y, axes='sxyz')
        else:
            # Start controlling
            if msg.buttons[BUTTON_START]:
                # Zero roll and pitch
                #TODO: This should be using odometry, instead its just leveling out.
                r, p, y = quat2euler([1,0,0,0], 'sxyz')
                r, p = 0, 0
                self.desired_orientation = euler2quat(r, p, y, axes='sxyz')

                # Submerge if not submerged. Else stop the bot
                #TODO: This should be using odometry, instead its just going 1 meter down.
                desired_position = (0,0,-1)
                if desired_position[2] > self.START_DEPTH:
                    desired_position[2] = self.START_DEPTH
                    lin_msg = ControllerCommand()
                    lin_msg.setpoint_vect = Vector3(float(desired_position[0]), float(desired_position[1]), float(desired_position[2]))
                    lin_msg.mode = ControllerCommand.POSITION
                    self.lin_pub.publish(lin_msg)
                else:
                    lin_msg = ControllerCommand()
                    lin_msg.setpoint_vect = Vector3()
                    lin_msg.mode = ControllerCommand.VELOCITY
                    self.lin_pub.publish(lin_msg)

                self.enabled = True
        
        self.software_kill_pub.publish(self.kill_msg)

    def odom_cb(self, msg):
        if self.enabled:
            dt = (self.get_clock().now() - self.last_odom_msg).nanoseconds / 1e9
            self.odom = msg
            rotation = euler2quat(*(self.ang_vel * dt), axes='sxyz')
            self.desired_orientation = qmult(self.desired_orientation, rotation)
            ang_msg = ControllerCommand()
            ang_msg.setpoint_quat = Quaternion(w=float(self.desired_orientation[0]), x=float(self.desired_orientation[1]), y=float(self.desired_orientation[2]), z=float(self.desired_orientation[3]))
            ang_msg.mode = ControllerCommand.POSITION
            self.ang_pub.publish(ang_msg)

        self.last_odom_msg = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)

    node = PS3Teleop()

    rclpy.spin(node)

if __name__=="__main__":
    main(sys.argv)
