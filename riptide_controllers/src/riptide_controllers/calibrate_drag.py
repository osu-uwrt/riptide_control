#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Twist
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from riptide_msgs2.action import CalibrateDragNew
from std_msgs.msg import Float32MultiArray, Empty

from transforms3d import euler
from queue import Queue
from math import pi, sqrt
import numpy as np
import csv, time, yaml, os

class CalibrateDragNewActionServer(Node):

    _result = CalibrateDragNew.Result()
    _goal = CalibrateDragNew.Goal()

    def __init__(self):
        super().__init__('calibrate_drag_new')
        self.declare_parameter("vehicle_config", rclpy.Parameter.Type.STRING)

        #self.twist_pub = self.create_publisher(Twist, "twist", qos_profile_system_default)
        #change body_force back to active_control when new thruster solver is integrated
        self.force_pub = self.create_publisher(Twist,"controller/active_body_force", qos_profile_system_default)
        
        #TODO: make trigger topic
        self.trigger_sub = self.create_subscription(Empty, "trigger", self.trigger_cb, qos_profile_system_default, callback_group = ReentrantCallbackGroup())
        
        self.odometry_sub = self.create_subscription(Odometry, "odometry/filtered", self.odometry_cb, qos_profile_system_default, callback_group=ReentrantCallbackGroup())
        self.odometry_queue = Queue(1)

        self.running = False
        self.paused = False

        self.csvPath = os.path.expanduser("~/osu-uwrt/dragCal.csv")
        if not os.path.exists(self.csvPath):
            self.get_logger().info(f"Proposed csv path {self.csvPath} does not exist; falling back to home directory")
            self.csvPath = os.path.expanduser("~/dragCal.csv")

        #self.param_set_client = self.create_client(SetParameters, "controller/set_parameters")
        #self.param_set_client.wait_for_service()

        self._action_server = ActionServer(
            self,
            CalibrateDragNew,
            'calibrate_drag_new',
            self.execute_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
    
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
        self.get_logger().info("Received request to cancel the action")
        self.running = False
        return CancelResponse.ACCEPT
    
    #Subscriber Functions

    def odometry_cb(self, msg):
        if self.odometry_queue.full():
            self.odometry_queue.get_nowait()
            
        self.odometry_queue.put_nowait(msg)

    def wait_for_odometry_msg(self):
        # Since the queue size is 1, if it has stuff in it just read to clear
        if not self.odometry_queue.empty():
            self.odometry_queue.get_nowait()
            assert self.odometry_queue.empty()
        
        return self.odometry_queue.get(True)
    
    def trigger_cb(self, msg):
        self.paused = False

    #Publisher functions
    def publish_force(self, value, axis):
        msg = Twist()
        temp = [0.0,0.0,0.0,0.0,0.0,0.0]
        temp[axis] = float(value)
        msg.linear = Vector3(x=temp[0], y=temp[1], z=temp[2])
        msg.angular = Vector3(x=temp[3], y=temp[4], z=temp[5])
        self.force_pub.publish(msg)

    #def publish_twist(self):
    #    msg = self.wait_for_odometry_msg().twist
    #    self.twist_pub.publish(msg)

    #Execution Functions

    #Run data collection along all 6 axes, write force and velocity data to columns in a csv
    def execute_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Starting drag calibration")
        
        #get start axis from goal request
        self._goal = goal_handle.request
        firstAxis = self._goal.start_axis

        #Create file if not exists
        try:
            with open(self.csvPath, "x") as csvfile:
                csvfile.close()
        except:
            a = 0

        with open(self.csvPath, "a", newline = "") as csvfile:
    
            writer = csv.writer(csvfile)
            
            #record new data
            for currentAxis in range(firstAxis,6):
                if not self.running:
                    self.get_logger().info("Action canceled.")
                    goal_handle.canceled()
                    return self._result
                    
                axisData = self.collect_data(axis = currentAxis)
                header = np.array([currentAxis])

                writer.writerow(np.append(header, np.flip(axisData)))

            csvfile.close()

        self.running = False
        goal_handle.succeed()
        return self._result
  
    #collect (net force, terminal velocity) data along a given axis, returns as numpy matrix with columns: force | velocity
    def collect_data(self, axis):
        force_data = [-49,-42,-35,-28,-21,-14,-7] if (axis < 2) else [-21,-18,-15,-12,-9,-6,-3]
        vel_data = [0,0,0,0,0,0,0]

        for i in range(len(force_data)):
            vel_data[i] = self.run_until_stable(force_data[i], axis)
            
            self.publish_force(0,axis)
            time.sleep(1)
            
            self.paused = True
            
            self.get_logger().info("Axis: %d, Datapoint: %d finished\n" % (axis, i))

            while self.paused and self.running:
                time.sleep(0.5)
            
            if not self.running:
                break
        
        return np.abs(np.array(vel_data))
    
    #Run at a specified force along given axis until velocity stabilizes (reaches terminal velocity)    
    def run_until_stable(self, force, axis):
        self.get_logger().info(f"Running until stable with force {force} on axis {axis}")
        self.get_logger().info("Waiting for odometry message")
        self.wait_for_odometry_msg()
        self.get_logger().info("Got odometry")
        
        self.publish_force(force, axis)
        
        stepTime = 0.1
        precision = 0.08
        stableSteps, stableStepsRequired = 0, 20
        vel, velPrev = 0, 0

        time.sleep(2)

        while stableSteps < stableStepsRequired and self.running:
            self.publish_force(force, axis)
            
            #Get velocity from Odometry topic
            twist = [
                lambda odom: odom.twist.twist.linear.x,
                lambda odom: odom.twist.twist.linear.y,
                lambda odom: odom.twist.twist.linear.z,
                lambda odom: odom.twist.twist.angular.x,
                lambda odom: odom.twist.twist.angular.y,
                lambda odom: odom.twist.twist.angular.z
            ]
            
            vel = twist[axis](self.wait_for_odometry_msg())
            
            velPrev = vel

            #Check if velocity ratio was stable within precision over interval
            if abs(vel - velPrev) < precision:
                stableSteps += 1
            else:
                stableSteps = 0
            
            time.sleep(stepTime)
        
        return abs(vel)

def main(args=None):
    rclpy.init(args=args)

    drag_cal_action_server = CalibrateDragNewActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(drag_cal_action_server, executor=executor)

    drag_cal_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()