#!/usr/bin/env python3

import asyncio
from queue import Queue
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float32, Bool, UInt16, Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from riptide_msgs2.msg import ControllerCommand
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from tf_transformations import quaternion_from_euler
from time import sleep


class LilTankTeleopNode(Node):
    def __init__(self):
        super().__init__("liltank_teleop_node")
        
        self.joySub = self.create_subscription(Joy, "/joy", self.onJoy, 10)        
        self.linPub = self.create_publisher(ControllerCommand, "/liltank/controller/linear", 10)
        self.angPub = self.create_publisher(ControllerCommand, "/liltank/controller/angular", 10)
            
    def onJoy(self, msg: Joy):
        angCmd = ControllerCommand()
        angCmd.mode = ControllerCommand.FEEDFORWARD
        angCmd.setpoint_vect.z = msg.axes[0]
        
        linCmd = ControllerCommand()
        linCmd.mode = ControllerCommand.FEEDFORWARD
        lt = (msg.axes[2] if msg.axes[2] < 0 else 0.0) * -1
        rt = (msg.axes[5] if msg.axes[5] < 0 else 0.0) * -1
        linCmd.setpoint_vect.x = rt - lt
        linCmd.setpoint_vect.z = msg.axes[7]
        
        self.angPub.publish(angCmd)
        self.linPub.publish(linCmd)
        

def main(args = None):
    rclpy.init()
    node = LilTankTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
