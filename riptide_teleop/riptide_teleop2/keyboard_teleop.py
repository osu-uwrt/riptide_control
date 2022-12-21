#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Empty

import transforms3d
import numpy as np
import math
import yaml

from pynput.keyboard import Key, Listener, KeyCode

def msgToNumpy(msg):
    if hasattr(msg, "w"):
        return np.array([msg.w, msg.x, msg.y, msg.z])
    return np.array([msg.x, msg.y, msg.z])

class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__("KeyboardTeleop")
        
        self.get_logger().info("p to pause, x to kill")
        self.get_logger().info("")
        self.get_logger().info("w,a,s,d\t\t\t-> y & z linear movement")
        self.get_logger().info("up,down,left,right\t-> angular z and linear x")
        self.get_logger().info("i,j,k,l\t\t\t-> y & z angular movement")
        self.get_logger().info("")
        self.get_logger().info("Keyboard Teleop disabled. Press e to enable")
        
        self.declare_parameter('vehicle_config', '/config/puddles.yaml')
        self._vehicle_config_path = self.get_parameter("vehicle_config").value
        with open(self._vehicle_config_path, 'r') as stream:
            config = yaml.safe_load(stream)
            self.max_linear_velocity = config['controller']['linear']['max']['velocity']
            self.max_angular_velocity = config['controller']['angular']['max']['velocity']

        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, 10)
        self.receivedOdom = False
        self.odom_msg = Odometry()
        
        self.lin_vel_pub = self.create_publisher(Vector3, "linear_velocity", 10)
        self.orientation_pub = self.create_publisher(Quaternion, "orientation", 10)
        self.position_pub = self.create_publisher(Vector3, "position", 10)
        self.off_pub = self.create_publisher(Empty, "off", 10)

        self.START_DEPTH = -1
        self.last_odom_msg_time = self.get_clock().now()
        self.last_linear_velocity = np.zeros(3)
        self.desired_orientation = np.array([0, 0, 0, 1.0])
        self.enabled = False
        self.keys = {}

    
    def start(self):
        self.get_logger().info("Waiting for odometry/filtered...")
        while not self.receivedOdom:
            rclpy.spin_once(self)
            
        self.get_logger().info("odometry/filtered received.")
            
        #flattens robot
        r, p, y = transforms3d.euler.quat2euler((msgToNumpy(self.odom_msg.pose.pose.orientation)), axes='sxyz')
        r, p = 0, 0
        self.desired_orientation = transforms3d.euler.euler2quat(r, p, y, axes='sxyz')
        desired_position = msgToNumpy(self.odom_msg.pose.pose.position)
        if desired_position[2] > self.START_DEPTH:
            desired_position[2] = self.START_DEPTH
        self.position_pub.publish(Vector3(float(desired_position[0]), float(desired_position[1]), float(desired_position[2])))
        self.enabled = True
        self.get_logger().info("Keyboard Teleop enabled")

    def stop(self):
        linear_velocity = Vector3()
        self.lin_vel_pub.publish(linear_velocity)
        self.last_linear_velocity = linear_velocity
        self.enabled = False
        self.get_logger().info("Keyboard Teleop disabled. Press e to enable")

    def on_press(self, key):
        self.keys[key] = True
        if self.enabled:
            if key == KeyCode.from_char("p"):
                self.stop()
            if key == KeyCode.from_char("x"):
                self.stop()
                self.off_pub.publish(Empty())
            if key == KeyCode.from_char("0"):
                r, p, y = transforms3d.euler.quat2euler(self.desired_orientation, axes='sxyz')
                r, p = 0, 0
                self.desired_orientation = transforms3d.euler.euler2quat(r, p, y, axes='sxyz')
        else:
            if key == KeyCode.from_char("e"):
                self.start()

    def on_release(self, key):
        self.keys[key] = False

    def odom_cb(self, msg):
        self.receivedOdom = True
        self.odom_msg = msg
        
        if self.enabled:
            linear_velocity = Vector3()
            ang_vel = np.zeros(3)

            if Key.down in self.keys and self.keys[Key.down]:
                linear_velocity.x = -self.max_linear_velocity[0]
            if Key.up in self.keys and self.keys[Key.up]:
                linear_velocity.x = self.max_linear_velocity[0]
            if Key.right in self.keys and self.keys[Key.right]:
                ang_vel[2] = -self.max_angular_velocity[2]
            if Key.left in self.keys and self.keys[Key.left]:
                ang_vel[2] = self.max_angular_velocity[2]
            if KeyCode.from_char("w") in self.keys and self.keys[KeyCode.from_char("w")]:
                linear_velocity.z = self.max_linear_velocity[2]
            if KeyCode.from_char("s") in self.keys and self.keys[KeyCode.from_char("s")]:
                linear_velocity.z = -self.max_linear_velocity[2]
            if KeyCode.from_char("a") in self.keys and self.keys[KeyCode.from_char("a")]:
                linear_velocity.y = self.max_linear_velocity[1]
            if KeyCode.from_char("d") in self.keys and self.keys[KeyCode.from_char("d")]:
                linear_velocity.y = -self.max_linear_velocity[1]
            if KeyCode.from_char("i") in self.keys and self.keys[KeyCode.from_char("i")]:
                ang_vel[1] = self.max_angular_velocity[1]
            if KeyCode.from_char("k") in self.keys and self.keys[KeyCode.from_char("k")]:
                ang_vel[1] = -self.max_angular_velocity[1]
            if KeyCode.from_char("j") in self.keys and self.keys[KeyCode.from_char("j")]:
                ang_vel[0] = -self.max_angular_velocity[0]
            if KeyCode.from_char("l") in self.keys and self.keys[KeyCode.from_char("l")]:
                ang_vel[0] = self.max_angular_velocity[0]

            dt = (self.get_clock().now() - self.last_odom_msg_time).to_sec()
            rotation = transforms3d.euler.euler2quat(*(ang_vel * dt))
            self.desired_orientation = transforms3d.quaternions.qmult(self.desired_orientation, rotation)
            self.orientation_pub.publish(Quaternion(w=float(self.desired_orientation[0]), x=float(self.desired_orientation[1]), y=float(self.desired_orientation[2]), z=float(self.desired_orientation[3])))

            if not np.array_equal(self.last_linear_velocity, msgToNumpy(linear_velocity)):
                self.lin_vel_pub.publish(linear_velocity)
                self.last_linear_velocity = msgToNumpy(linear_velocity)

        self.last_odom_msg_time = self.get_clock().now()
        
def main(args=None):
    rclpy.init()

    keyboardTeleop = KeyboardTeleop()
    with Listener(
        on_press=keyboardTeleop.on_press,
        on_release=keyboardTeleop.on_release) as listener:

        rclpy.spin(keyboardTeleop)