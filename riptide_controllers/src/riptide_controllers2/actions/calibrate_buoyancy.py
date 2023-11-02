#! /usr/bin/env python3

# Determines the buoyancy parameter of the robot.
# Assumes the robot is upright

from asyncio import current_task
import rclpy
import time
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from queue import Queue
from riptide_msgs2.msg import ControllerCommand

from geometry_msgs.msg import Vector3, Quaternion, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from riptide_msgs2.action import CalibrateBuoyancy

from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_inverse, quaternion_multiply

import math
import yaml
import numpy as np

WATER_DENSITY = 1000
GRAVITY = 9.81

def msg_to_numpy(msg):
    """Converts a Vector3 or Quaternion message to its numpy counterpart"""
    if hasattr(msg, "w"):
        return np.array([msg.x, msg.y, msg.z, msg.w])
    return np.array([msg.x, msg.y, msg.z])

def changeFrame(orientation, vector, w2b = True):
    """Converts vector into other frame from orientation quaternion. The w2b parameter will
     determine if the vector is converting from world to body or body to world"""

    vector = np.append(vector, 0)
    if w2b:
        orientation = quaternion_inverse(orientation)
    orientationInv = quaternion_inverse(orientation)
    newVector = quaternion_multiply(orientation, quaternion_multiply(vector, orientationInv))
    return newVector[:3]


class CalibrateBuoyancyAction(Node):

    _result = CalibrateBuoyancy.Result()
    INITIAL_PARAM_NAMES = [
        'volume', 'cob_x', 'cob_y', 'cob_z',
        'linear_damping_x', 'linear_damping_y',
        'linear_damping_z', 'linear_damping_rot_x',
        'linear_damping_rot_y', 'linear_damping_rot_z',
        'quadratic_damping_x', 'quadratic_damping_y',
        'quadratic_damping_z', 'quadratic_damping_rot_x',
        'quadratic_damping_rot_y', 'quadratic_damping_rot_z'
    ]

    def __init__(self):
        super().__init__('calibrate_buoyancy')
        self.declare_parameter("vehicle_config", rclpy.Parameter.Type.STRING)

        self.linear_pub = self.create_publisher(ControllerCommand, "controller/linear", qos_profile_system_default)
        self.angular_pub = self.create_publisher(ControllerCommand, "controller/angular", qos_profile_system_default)
        
        self.odometry_sub = self.create_subscription(Odometry, "odometry/filtered", self.odometry_cb, qos_profile_system_default)
        self.odometry_queue = Queue(1)
        self.requested_accel_sub = self.create_subscription(JointState, "controller/state", self.requested_accel_cb, qos_profile_system_default)
        self.requested_accel_queue = Queue(1)

        # Get the mass and COM
        with open(self.get_parameter('vehicle_config').value, 'r') as stream:
            vehicle = yaml.safe_load(stream)
            self.mass = vehicle['mass']
            self.com = np.array(vehicle['com'])
            self.inertia = np.array(vehicle['inertia'])

        self.running = False

        self.param_get_client = self.create_client(GetParameters, "controller/get_parameters")
        self.param_set_client = self.create_client(SetParameters, "controller/set_parameters")
        self.param_get_client.wait_for_service()
        self.param_set_client.wait_for_service()

        self._action_server = ActionServer(
            self,
            CalibrateBuoyancy,
            'calibrate_buoyancy',
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
        return CancelResponse.ACCEPT


    ##############################
    # Message Wait Functions
    ##############################

    def odometry_cb(self, msg: Odometry) -> None:
        if not self.odometry_queue.full():
            self.odometry_queue.put_nowait(msg)

    def wait_for_odometry_msg(self) -> Odometry:
        # Since the queue size is 1, if it has stuff in it just read to clear
        if not self.odometry_queue.empty():
            self.odometry_queue.get_nowait()
            assert self.odometry_queue.empty()
        
        return self.odometry_queue.get(True)

    def requested_accel_cb(self, msg: JointState) -> None:
        if not self.requested_accel_queue.full():
            internal_twist = Twist
            def getVect(name: str):
                vect = Vector3()
                vect.x = msg.effort[msg.name.index(f'{name}_x')]
                vect.y = msg.effort[msg.name.index(f'{name}_y')]
                vect.z = msg.effort[msg.name.index(f'{name}_z')]
                return vect

            internal_twist.linear = getVect('lin')
            internal_twist.angular = getVect('ang')

            self.requested_accel_queue.put_nowait(internal_twist)

    def wait_for_requested_accel_msg(self) -> Twist:
        # Since the queue size is 1, if it has stuff in it just read to clear
        if not self.requested_accel_queue.empty():
            self.requested_accel_queue.get_nowait()
            assert self.requested_accel_queue.empty()
        
        return self.requested_accel_queue.get(True)

    ##############################
    # Parameter Utility Functions
    ##############################
    def load_initial_controller_config(self):
        request = GetParameters.Request()
        request.names = self.INITIAL_PARAM_NAMES
        response: GetParameters.Response = self.param_get_client.call(request)
        if len(response.values) != len(self.INITIAL_PARAM_NAMES):
            self.get_logger().error("Unable to retrieve all requested parameters")
            return False
        
        self.initial_config = {}
        for i in range(len(response.values)):
            if response.values[i].type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                self.initial_config[self.INITIAL_PARAM_NAMES[i]] = list(response.values[i].double_array_value)
            else:
                self.initial_config[self.INITIAL_PARAM_NAMES[i]] = response.values[i].double_value

        return True

    def update_controller_config(self, config: dict):
        parameters = []
        for entry in config:
            param = Parameter()
            param.name = entry
            param_value = ParameterValue()
            if type(config[entry]) == list:
                param_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param_value.double_array_value = list(map(float, config[entry]))
            else:
                param_value.type = ParameterType.PARAMETER_DOUBLE
                param_value.double_value = float(config[entry])
            param.value = param_value
            parameters.append(param)
        
        request = SetParameters.Request()
        request.parameters = parameters
        response: SetParameters.Response = self.param_set_client.call(request)
        
        if len(response.results) != len(parameters):
            self.get_logger().error("Unable to set all requested parameters")
            return False
        
        for entry in response.results:
            if not entry.successful:
                self.get_logger().error("Failed to set parameter: " + str(entry.reason))
                return False
        
        return True

    ##############################
    # Calibration Functions
    ##############################

    def tune(self, goal_handle, initial_value, get_adjustment, apply_change, num_samples=10, delay=4):
        """Tunes a parameter of the robot"""
        current_value = np.array(initial_value)
        last_adjustment = np.zeros_like(current_value)
        converged = np.zeros_like(current_value)

        while not np.all(converged):
            # Wait for equilibrium
            time.sleep(delay)

            # Average a few samples
            average_adjustment = 0
            for _ in range(num_samples):
                average_adjustment += get_adjustment() / num_samples
                time.sleep(0.2)

            # Apply change
            current_value += average_adjustment
            apply_change(current_value)

            # Check if the value has converged
            converged = np.logical_or(converged, average_adjustment * last_adjustment < 0)
            last_adjustment = average_adjustment

            if self.check_preempted(goal_handle):
                return None

        return current_value

      
    def execute_cb(self, goal_handle):
        self.running = True
        # Start reconfigure server and get starting config
        if not self.load_initial_controller_config():
            goal_handle.abort()
            self.running = False
            return CalibrateBuoyancy.Result()
        
        # Set variables to defaults
        self.get_logger().info("Starting buoyancy calibration")
        volume = self.mass / WATER_DENSITY
        cob = np.copy(self.com)

        # Reset parameters to default
        self.update_controller_config({
            "volume": volume, 
            "cob_x": cob[0],
            "cob_y": cob[1],
            "cob_z": cob[2],
            "linear_damping_x": 0.0,
            "linear_damping_y": 0.0,
            "linear_damping_z": 0.0,
            "linear_damping_rot_x": 0.0,
            "linear_damping_rot_y": 0.0,
            "linear_damping_rot_z": 0.0,
            "quadratic_damping_x": 0.0,
            "quadratic_damping_y": 0.0,
            "quadratic_damping_z": 0.0,
            "quadratic_damping_rot_x": 0.0,
            "quadratic_damping_rot_y": 0.0,
            "quadratic_damping_rot_z": 0.0
        })

        # Submerge
        odom_msg = self.wait_for_odometry_msg()
        current_position = msg_to_numpy(odom_msg.pose.pose.position)
        current_orientation = msg_to_numpy(odom_msg.pose.pose.orientation)

        linear_command = ControllerCommand(mode=ControllerCommand.POSITION, setpoint_vect=Vector3(x=current_position[0], y=current_position[1], z=-1.5))
        self.linear_pub.publish(linear_command)

        _, _, yaw = euler_from_quaternion(current_orientation)
        x,y,z,w = quaternion_from_euler(0, 0, yaw)

        angularCommand = ControllerCommand(mode=ControllerCommand.POSITION, setpoint_quat=Quaternion(w=w, x=x, y=y, z=z))
        self.angular_pub.publish(angularCommand)

        # Wait for equilibrium
        time.sleep(10)

        # Volume adjustment function
        def get_volume_adjustment():
            body_force = self.mass * msg_to_numpy(self.wait_for_requested_accel_msg().linear)
            orientation = msg_to_numpy(self.wait_for_odometry_msg().pose.pose.orientation)
            world_z_force = changeFrame(orientation, body_force, w2b=False)[2]

            return -world_z_force / WATER_DENSITY / GRAVITY

        # Tune volume
        volume = self.tune(goal_handle,
            volume, 
            get_volume_adjustment, 
            lambda v: self.update_controller_config({"volume": v})
        )
        if volume == None:
            return CalibrateBuoyancy.Result()

        self.get_logger().info("Volume calibration complete")
        buoyant_force = volume * WATER_DENSITY * GRAVITY

        # COB adjustment function
        def get_cob_adjustment():
            accel = msg_to_numpy(self.wait_for_requested_accel_msg().angular)
            torque = self.inertia * accel
            orientation = msg_to_numpy(self.wait_for_odometry_msg().pose.pose.orientation)
            body_force_z = changeFrame(orientation, np.array([0, 0, buoyant_force]))[2]

            adjustment_x = torque[1] / body_force_z
            adjustment_y = -torque[0] / body_force_z

            return np.array([adjustment_x, adjustment_y])

        # Tune X and Y COB
        if self.tune(goal_handle,
            cob[:2], 
            get_cob_adjustment, 
            lambda cob_local: self.update_controller_config({"cob_x": cob_local[0], "cob_y": cob_local[1], "cob_z": cob[2]}),
            num_samples = 2,
            delay = 1
        ) is None:
            return CalibrateBuoyancy.Result()

        self.get_logger().info("Buoyancy XY calibration complete")

        # Adjust orientation
        x,y,z,w = quaternion_from_euler(0, -math.pi / 4, yaw)
        angularCommand = ControllerCommand(mode=ControllerCommand.POSITION, setpoint_quat=Quaternion(w=w, x=x, y=y, z=z))
        self.angular_pub.publish(angularCommand)

        time.sleep(3)

        # Z COB function
        def get_cob_z_adjustment():
            accel = msg_to_numpy(self.wait_for_requested_accel_msg().angular)
            torque = self.inertia * accel
            orientation = msg_to_numpy(self.wait_for_odometry_msg().pose.pose.orientation)
            body_force_x = changeFrame(orientation, np.array([0, 0, buoyant_force]))[0]

            adjustment = -torque[1] / body_force_x

            return adjustment

        # Tune Z COB
        if self.tune(goal_handle,
            cob[2], 
            get_cob_z_adjustment, 
            lambda z: self.update_controller_config({"cob_x": cob[0], "cob_y": cob[1], "cob_z": z})
        ) is None:
            return CalibrateBuoyancy.Result()

        try:
            self.get_logger().info("Calibration complete")
            self.cleanup()
            self._result.buoyant_force = buoyant_force
            self._result.center_of_buoyancy = list(cob)
        except:
            import traceback
            tb = traceback.format_exc()
            self.get_logger().warn("Crashed: %s" % str(tb))
            raise

        self.running = False
        goal_handle.succeed()
        return self._result

    def check_preempted(self, goal_handle):
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Preempted Calibration')
            self.cleanup()
            goal_handle.canceled()
            self.running = False
            return True

    def cleanup(self):
        self.update_controller_config({
            "linear_damping_x": self.initial_config['linear_damping_x'],
            "linear_damping_y": self.initial_config['linear_damping_y'],
            "linear_damping_z": self.initial_config['linear_damping_z'],
            "linear_damping_rot_x": self.initial_config['linear_damping_rot_x'],
            "linear_damping_rot_y": self.initial_config['linear_damping_rot_y'],
            "linear_damping_rot_z": self.initial_config['linear_damping_rot_z'],
            "quadratic_damping_x": self.initial_config['quadratic_damping_x'],
            "quadratic_damping_y": self.initial_config['quadratic_damping_y'],
            "quadratic_damping_z": self.initial_config['quadratic_damping_z'],
            "quadratic_damping_rot_x": self.initial_config['quadratic_damping_rot_x'],
            "quadratic_damping_rot_y": self.initial_config['quadratic_damping_rot_y'],
            "quadratic_damping_rot_z": self.initial_config['quadratic_damping_rot_z']
        })

        # turn the controller off
        off_msg = ControllerCommand()
        off_msg.mode = ControllerCommand.DISABLED
        self.linear_pub.publish(off_msg)
        self.angular_pub.publish(off_msg)


def main(args=None):
    rclpy.init(args=args)

    thruster_test_action_server = CalibrateBuoyancyAction()

    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_test_action_server, executor=executor)

    thruster_test_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
