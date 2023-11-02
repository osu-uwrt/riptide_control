#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default # can replace this with others
from rclpy.action import ActionClient

from riptide_msgs2.action import FollowTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory

class TrajTestNode(Node):

    def __init__(self):
        super().__init__('trajectory_action_client')
        self._action_client = ActionClient(self, FollowTrajectory, 'follow_trajectory')

    def send_goal(self):
        goal_msg = FollowTrajectory.Goal()
        traj = MultiDOFJointTrajectory()
        traj.points = []
        goal_msg.trajectory = traj

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = TrajTestNode()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main() 
