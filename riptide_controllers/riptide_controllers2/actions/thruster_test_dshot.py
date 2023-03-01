#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from riptide_msgs2.msg import DshotCommand
from riptide_msgs2.action import ThrusterTest
from std_msgs.msg import Float32MultiArray

import yaml

import time

#the factor to publish the thruster test at 
THRUSTER_NEUTRAL_FORCE = 0.0
THRUSTER_TEST_FORCE = 1.3333

DSHOT_NEUTRAL_VALUE = 0
DSHOT_TEST_FACTOR = 0.05

class ThrusterTestActionServer(Node):
    THRUSTER_PERCENT = 0.05

    def __init__(self):
        super().__init__('thruster_test')
        self.declare_parameter("vehicle_config", rclpy.Parameter.Type.STRING)

        self.Dshot_pub = self.create_publisher(DshotCommand ,"command/dshot", qos_profile_sensor_data)
        self.thruster_forces_pub = self.create_publisher(Float32MultiArray ,"thruster_forces", qos_profile_system_default)

        # Get the mass and COM
        with open(self.get_parameter('vehicle_config').value, 'r') as stream:
            self.vehicle_file = yaml.safe_load(stream)
            self.num_thrusters = len(self.vehicle_file["thrusters"])

        #get dshot max value
        self.Dshot_max = DshotCommand.DSHOT_MAX
        self.running = False

        self._action_server = ActionServer(
            self,
            ThrusterTest,
            'thruster_test',
            self.execute_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    ##############################
    # Protections 
    
    def destroy(self):
        self.destroy_node()
        self._action_server.destroy()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        if self.running:
            return GoalResponse.REJECT
        else:
            self.running = True
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        return CancelResponse.ACCEPT

    def publish_dshot(self, values):
        #publish dshot values
        msg = DshotCommand()
        msg.values = values
        self.Dshot_pub.publish(msg)

    def publish_forces(self, forces):
        msg = Float32MultiArray()
        msg.data = forces
        self.thruster_forces_pub.publish(msg)

    def execute_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Starting ThrusterTest Action")
        Dshot = [DSHOT_NEUTRAL_VALUE] * self.num_thrusters
        thruster_forces = [THRUSTER_NEUTRAL_FORCE] * self.num_thrusters

        while True:
            for i in range(self.num_thrusters):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Preempted ThrusterTest Action')
                    self.publish_dshot([DSHOT_NEUTRAL_VALUE] * self.num_thrusters)
                    self.publish_forces([THRUSTER_NEUTRAL_FORCE] * self.num_thrusters)

                    self.running = False
                    goal_handle.canceled()
                    return ThrusterTest.Result()

                thruster_type = self.vehicle_file["thrusters"][i]["type"]
                thruster_name = self.vehicle_file["thrusters"][i]["name"]

                self.get_logger().info(f'Testing {thruster_name} Thruster ({i+1})')

                thruster_forces[i] = THRUSTER_TEST_FORCE
                if thruster_type == 0:
                    Dshot[i] = int(self.Dshot_max * DSHOT_TEST_FACTOR)
                else:
                    Dshot[i] = -int(self.Dshot_max * DSHOT_TEST_FACTOR)

                for _ in range(300):
                    self.publish_dshot(Dshot)
                    self.publish_forces(thruster_forces)
                    time.sleep(0.01)
                
                Dshot[i] = DSHOT_NEUTRAL_VALUE
                thruster_forces[i] = THRUSTER_NEUTRAL_FORCE

        #should never reach this point in the code
        self.get_logger().info("ThrustTest succeeded")
        goal_handle.succeed()
        self.running = False
        return ThrusterTest.Result()

def main(args=None):
    rclpy.init(args=args)

    thruster_test_action_server = ThrusterTestActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_test_action_server, executor=executor)

    thruster_test_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()