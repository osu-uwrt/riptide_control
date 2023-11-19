#!/usr/bin/env python3

# thruster_wrench_publisher node
#
# Input topics:
#   thruster_forces: Array containing how hard each thruster will push.
#
# Output topics:
#   thruster_wrenches/thruster_0...n: A WrenchStamped in the thruster frame with the thruster's force vector
#
# This node republishes the thruster_forces as individual WrenchStamped topics so 
# thruster force vectors can be visualized in rviz

import rclpy
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default # can replace this with others

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

import yaml


class ThrusterWrenchPublisher(Node):

    def __init__(self):
        super().__init__('thruster_wrench_publisher')

        self.create_subscription(Float32MultiArray, "thruster_forces", self.force_cb, qos_profile_system_default)

        self.declare_parameter("robot", "")
        self.tf_namespace = self.get_parameter("robot").value
        self.stamp = Time()

        # Load thruster info
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if(config_path == ''):
            self.get_logger().fatal("vehicle config file param not set or empty, exiting")

        with open(config_path, 'r') as stream:
            config_file = yaml.safe_load(stream)
        thruster_info = config_file['thrusters']

        self.thruster_wrench_publishers: 'list[Publisher]' = []
        for i in range(len(thruster_info)):
            wrench_pub = self.create_publisher(WrenchStamped, f'thruster_wrenches/thruster_{i}', qos_profile_system_default)
            self.thruster_wrench_publishers.append(wrench_pub)

    def force_cb(self, msg: Float32MultiArray):
        if len(msg.data) != len(self.thruster_wrench_publishers):
            self.get_logger().error("thruster_forces message length ({0}) does not match expected number of thrusters ({1})".format(len(msg), len(self.thruster_wrench_publishers)))
            return

        for i in range(len(self.thruster_wrench_publishers)):
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = f'{self.tf_namespace}/thruster_{i}'
            wrench_msg.header.stamp = self.stamp.to_msg()
            wrench_msg.wrench.force.x = msg.data[i]

            self.thruster_wrench_publishers[i].publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterWrenchPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()