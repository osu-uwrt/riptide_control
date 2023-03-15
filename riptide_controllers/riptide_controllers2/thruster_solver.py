#!/usr/bin/env python3

# thruster_solver node
#
# Input topics:
#   net_force: The force the control system wants the robot to exert on the world to move
#
# Output topics:
#   thruster_forces: Array containing how hard each thruster will push. 
#
# This node works via optimization. A cost function is proposed that measures how optimal a set of thruster forces is.
# This function takes into account exerting the correct total forces and power consumption. This node will also ignore
# thrusters that are currently out of the water and solve with the thrusters that are in the water. On each new net_force
# message, the robot solves for the optimal thruster forces and publishes them

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data# can replace this with others

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Twist
from riptide_msgs2.msg import DshotCommand
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

import numpy as np
import yaml
from tf_transformations import euler_matrix
from scipy.optimize import minimize

from riptide_controllers2.Controllers import msgToNumpy


NUETRAL_DSHOT = 0

class ThrusterSolverNode(Node):

    def __init__(self):
        super().__init__('riptide_controllers2')

        self.create_subscription(Twist, "controller/body_force", self.force_cb, qos_profile_system_default)

        self.thruster_pub = self.create_publisher(Float32MultiArray, "thruster_forces", qos_profile_system_default)
        self.dshot_pub = self.create_publisher(DshotCommand ,"command/dshot", qos_profile_sensor_data)
        self.enabledSub = self.create_subscription(Bool, "state/kill", self.kill_cb, qos_profile_sensor_data)

        self.declare_parameter("robot", "")
        self.tf_namespace = self.get_parameter("robot").value

        # Load thruster info
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if(config_path == ''):
            self.get_logger().fatal("vehicle config file param not set or empty, exiting")

        with open(config_path, 'r') as stream:
            config_file = yaml.safe_load(stream)


        thruster_info = config_file['thrusters']
        self.thruster_coeffs = np.zeros((len(thruster_info), 6))
        self.thruster_types = np.zeros(len(thruster_info))
        com = np.array(config_file["com"])
        self.max_force = config_file["thruster"]["max_force"]
        self.dshot_file = config_file["thruster"]

        for i, thruster in enumerate(thruster_info):
            pose = np.array(thruster["pose"])
            rot_mat = euler_matrix(*pose[3:])
            body_force = np.dot(rot_mat, np.array([1, 0, 0, 0]))[:3]
            body_torque = np.cross(pose[:3]- com, body_force)

            self.thruster_coeffs[i, :3] = body_force
            self.thruster_coeffs[i, 3:] = body_torque   

            self.thruster_types[i] = config_file["thrusters"][i]["type"]   

        self.initial_condition = []
        self.bounds = []
        for i in range(len(thruster_info)):
            self.initial_condition.append(0)
            self.bounds.append((-self.max_force, self.max_force))
            
        self.initial_condition = tuple(self.initial_condition)
        self.bounds = tuple(self.bounds)

        self.power_priority = 0.001
        self.current_thruster_coeffs = np.copy(self.thruster_coeffs)

        self.start_time = None
        self.timer = self.create_timer(0.1, self.check_thrusters)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.dshot_max = DshotCommand.DSHOT_MAX
        self.dshot_min = DshotCommand.DSHOT_MIN

        self.WATER_LEVEL = 0
        
        self.enabled = False
        
    def kill_cb(self, msg: Bool):
        self.enabled = not msg.data
        

    def publish_pwm(self, forces):
        dshot_values = []

        #convert thruster force to dshot
        for i in range(self.thruster_coeffs.shape[0]):
            dshot = NUETRAL_DSHOT
               
            # robot is killed or no force is needed
            if(not self.enabled or abs(forces[i]) < self.dshot_file["MIN_THRUST"]):
                dshot = NUETRAL_DSHOT

            elif (forces[i] > 0 and forces[i] <= self.dshot_file["STARTUP_THRUST"]):
                if self.thruster_types[i] == 0:
                    dshot = (int)(self.dshot_file["SU_THRUST"]["POS_SLOPE"] * forces[i] + self.dshot_file["SU_THRUST"]["POS_YINT"])
                else:
                    dshot = (int)(-self.dshot_file["SU_THRUST"]["POS_SLOPE"] * forces[i] + self.dshot_file["SU_THRUST"]["NEG_YINT"])

            elif (forces[i] > 0 and forces[i] > self.dshot_file["STARTUP_THRUST"]):
                if self.thruster_types[i] == 0:
                    dshot = (int)(self.dshot_file["THRUST"]["POS_SLOPE"] * forces[i] + self.dshot_file["THRUST"]["POS_YINT"])
                else:
                    dshot = (int)(-self.dshot_file["THRUST"]["POS_SLOPE"] * forces[i] + self.dshot_file["THRUST"]["NEG_YINT"])

            elif (forces[i] < 0 and forces[i] >= -self.dshot_file["STARTUP_THRUST"]):
                if self.thruster_types[i] == 0:
                    dshot = (int)(self.dshot_file["SU_THRUST"]["NEG_SLOPE"] * forces[i] + self.dshot_file["SU_THRUST"]["NEG_YINT"])
                else:
                    dshot = (int)(-self.dshot_file["SU_THRUST"]["NEG_SLOPE"] * forces[i] + self.dshot_file["SU_THRUST"]["POS_YINT"])

            elif (forces[i] < 0 and forces[i] < -self.dshot_file["STARTUP_THRUST"]):
                if self.thruster_types[i] == 0:
                    dshot = (int)(self.dshot_file["THRUST"]["NEG_SLOPE"] * forces[i] + self.dshot_file["THRUST"]["NEG_YINT"])
                else:
                    dshot = (int)(-self.dshot_file["THRUST"]["NEG_SLOPE"] * forces[i] + self.dshot_file["THRUST"]["POS_YINT"])

            else:
                dshot = NUETRAL_DSHOT

            #ensure dshot is not out of bounds
            dshot = min(self.dshot_max, dshot)
            dshot = max(self.dshot_min, dshot)

            dshot_values.append(dshot)

        # Make the dshot message
        msg = DshotCommand()
        msg.values = dshot_values
        self.dshot_pub.publish(msg)


    # Timer callback which disables thrusters that are out of the water
    def check_thrusters(self):
        try:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            self.current_thruster_coeffs = np.copy(self.thruster_coeffs)
            for i in range(self.thruster_coeffs.shape[0]):
                self.get_logger().debug("transforming from world to %s/thruster_%d" % (self.tf_namespace, i))
                trans = self.tf_buffer.lookup_transform("world", "%s/thruster_%d" % (self.tf_namespace, i), Time())  
                if trans.transform.translation.z > self.WATER_LEVEL:
                    self.current_thruster_coeffs[i, :] = 0
        except Exception as ex:
            # Supress startup errors
            if (self.get_clock().now() - self.start_time) > Duration(seconds=1.0):
                self.get_logger().fatal(str(ex))

    # Cost function forcing the thruster to output desired net force
    def force_cost(self, thruster_forces, desired_state):
        residual = np.dot(self.current_thruster_coeffs.T, thruster_forces) - desired_state
        return np.sum(residual ** 2)
    
    def force_cost_jac(self, thruster_forces, desired_state):
        residual = np.dot(self.current_thruster_coeffs.T, thruster_forces) - desired_state
        return np.dot(self.current_thruster_coeffs, 2 * residual)

    # Cost function forcing thrusters to find a solution that is low-power
    def power_cost(self, thruster_forces):
        return np.sum(thruster_forces ** 2)

    def power_cost_jac(self, thruster_forces):
        return 2 * thruster_forces

    # Combination of other cost functions
    def total_cost(self, thruster_forces, desired_state):
        total_cost = self.force_cost(thruster_forces, desired_state)
        # We care about low power a whole lot less thus the lower priority
        total_cost += self.power_cost(thruster_forces) * self.power_priority
        return total_cost

    def total_cost_jac(self, thruster_forces, desired_state):
        total_cost_jac = self.force_cost_jac(thruster_forces, desired_state)
        total_cost_jac += self.power_cost_jac(thruster_forces) * self.power_priority
        return total_cost_jac

    def force_cb(self, msg):
        
        desired_state = np.zeros(6)
        desired_state[:3] = msgToNumpy(msg.linear)
        desired_state[3:] = msgToNumpy(msg.angular)

        # Optimize cost function to find optimal thruster forces
        res = minimize(self.total_cost, self.initial_condition, args=(desired_state), method='SLSQP', \
                        jac=self.total_cost_jac, bounds=self.bounds)

        if(self.enabled):
            # Warn if we did not find valid thruster forces
            fcost = self.force_cost(res.x, desired_state)
            if(fcost > 0.05):
                self.get_logger().warning(f"Unable to exert force, cost {round(fcost)} > 0.05 solution: {res.x}, des_state{desired_state}")
            elif(not res.success):
                self.get_logger().warning("Unable to exert requested force, could not solve jacobean")

        data = []
        for val in res.x :
            if abs(val) < self.dshot_file["MIN_THRUST"]:
                val = 0
            
            data.append(float(val))

        msg = Float32MultiArray()
        msg.data = data      

        self.publish_pwm(res.x)

        self.thruster_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterSolverNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()