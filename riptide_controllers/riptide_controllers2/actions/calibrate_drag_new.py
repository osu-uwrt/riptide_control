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
from riptide_msgs2.msg import ControllerCommand
from std_msgs.msg import Float32MultiArray, Empty

from transforms3d import euler
from queue import Queue
from math import pi, sqrt
import numpy as np
import csv, time, yaml, os

class CalibrateDragNewActionServer(Node):

    _result = CalibrateDragNew.Result()

    def __init__(self):
        super().__init__('calibrate_drag_new')
        self.declare_parameter("vehicle_config", rclpy.Parameter.Type.STRING)

        #self.twist_pub = self.create_publisher(Twist, "twist", qos_profile_system_default)
        self.force_pub = self.create_publisher(Float32MultiArray,"talos/controller/active_control", qos_profile_system_default)
        
        #TODO: make trigger topic
        self.trigger_sub = self.create_subscription(Empty, "talos/trigger/drag_cal_trigger", self.trigger_cb, qos_profile_system_default, callback_group = ReentrantCallbackGroup())
        
        self.odometry_sub = self.create_subscription(Odometry, "odometry/filtered", self.odometry_cb, qos_profile_system_default, callback_group=ReentrantCallbackGroup())
        self.odometry_queue = Queue(1)

        self.running = False
        self.paused = False

        self.csvPath = "dragCal.csv"

        self.param_set_client = self.create_client(SetParameters, "controller/set_parameters")
        self.param_set_client.wait_for_service()

        self._action_server = ActionServer(
            self,
            CalibrateDragNew,
            'calibrate_drag_new',
            self.execute_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
    
    def destroy(self):
        #write data to csv 
        with open(self.csvPath, "w", newline = "") as csvfile:
    
            writer = csv.writer(csvfile)
        
            self.csvData = self.csvData[:,1:]

            #writer.writerow(["X Force", "X Vel", "Y Force", "Y Vel", "Z Force", "Z Vel", "Roll Torque", "Roll", "Pitch Torque", "Pitch", "Yaw Torque", "Yaw", ])
            writer.writerows(self.csvData)

            csvfile.close()

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
        return CancelResponse.REJECT
    
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
        msg = Float32MultiArray()
        temp = [0,0,0,0,0,0]
        temp[axis] = value
        msg._data = temp
        self.force_pub.publish(msg)

    #def publish_twist(self):
    #    msg = self.wait_for_odometry_msg().twist
    #    self.twist_pub.publish(msg)

    #Execution Functions

    #Run data collection along all 6 axes, write force and velocity data to columns in a csv
    def execute_cb(self, goal_handle: ServerGoalHandle):
        
        firstAxis = 0
        csvData = np.empty(7)

        #Load saved data, full twelve-column files will be overwritten, not reloaded
        with open(self.csvPath, "r") as csvfile:
            if(os.path.getsize(self.csvPath) > 0):
                temp = np.genfromtxt(self.csvPath, delimiter=',')
                firstAxis = int(np.shape(temp)[1]/2) % 6
                if(firstAxis > 0):
                     csvData = np.column_stack((csvData,temp))
            csvfile.close()
            print(firstAxis)
        
        #record new data
        for axis in range(firstAxis,6):
            axisData = self.collect_data(self, axis)
            self.csvData = np.column_stack((self.csvData,axisData))
  
    #collect (net force, terminal velocity) data along a given axis, returns as numpy matrix with columns: force | velocity
    def collect_data(self, axis):
        force_data = [7,6,5,4,3,2,1]
        vel_data = []

        for i in range(len(force_data)):
            vel_data[i] = self.run_until_stable(self, force_data[i], axis)
            
            self.publish_force(0,axis)
            time.sleep(1)
            
            paused = True
            self.get_logger().info("Axis: %d, Datapoint: %d finished\n", axis, i)
            while paused:
                time.sleep(0.5)
        
        return np.column_stack((np.flip(np.array(force_data)), np.flip(np.array(vel_data))))
    
    #Run at a specified force along given axis until velocity stabilizes (reaches terminal velocity)    
    def run_until_stable(self, force, axis):
        self.publish_force(force, axis)
        
        stepTime = 0.1
        precision = 0.1
        stableSteps, stableStepsRequired = 0, 10
        vel, velPrev, velRatio = 0, 0, 0

        time.sleep(2)

        while stableSteps < stableStepsRequired:
            
            #Get velocity from Odometry topic
            twist= [
                self.wait_for_odometry_msg().twist.linear.x,
                self.wait_for_odometry_msg().twist.linear.y,
                self.wait_for_odometry_msg().twist.linear.z,
                self.wait_for_odometry_msg().twist.angular.x,
                self.wait_for_odometry_msg().twist.angular.y,
                self.wait_for_odometry_msg().twist.angular.z,
            ]
            
            vel = twist[axis]
            if(velPrev != 0):
                velRatio = vel/velPrev
            else:
                velRatio = 0
                
            velPrev = vel

            #Check if velocity ratio was stable within precision over interval
            if (velRatio < (1 + precision)) and (velRatio > (1 - precision)):
                stableSteps += 1
            else:
                stableSteps = 0
            
            time.sleep(stepTime)
        
        return vel

def main(args=None):
    rclpy.init(args=args)

    drag_cal_action_server = CalibrateDragNewActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(drag_cal_action_server, executor=executor)

    drag_cal_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()