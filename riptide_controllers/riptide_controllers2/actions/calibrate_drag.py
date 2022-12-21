#! /usr/bin/env python3
import rclpy
import time
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from queue import Queue

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Twist
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from riptide_msgs2.action import CalibrateDrag
from riptide_msgs2.msg import ControllerCommand

from transforms3d import euler
from math import pi
from math import sqrt
import numpy as np

# Z axis is a little off, load interia from puddles.yaml, and update dynamic reconfigure


class CalibrateDragActionServer(Node):

    _result = CalibrateDrag.Result()

    def __init__(self):
        super().__init__('calibrate_drag')
        self.declare_parameter("vehicle_config", rclpy.Parameter.Type.STRING)

        self.linear_pub = self.create_publisher(ControllerCommand, "controller/linear", qos_profile_system_default)
        self.angular_pub = self.create_publisher(ControllerCommand, "controller/angular", qos_profile_system_default)
        
        self.odometry_sub = self.create_subscription(Odometry, "odometry/filtered", self.odometry_cb, qos_profile_system_default, callback_group=ReentrantCallbackGroup())
        self.odometry_queue = Queue(1)

        self.requested_accel_sub = self.create_subscription(JointState, "controller/state", self.requested_accel_cb, qos_profile_system_default, callback_group=ReentrantCallbackGroup())
        self.requested_accel_queue = Queue(1)

        # Get the mass and COM
        with open(self.get_parameter('vehicle_config').value, 'r') as stream:
            vehicle = yaml.safe_load(stream)
            mass = vehicle['mass']
            rotational_inertia = np.array(vehicle['inertia'])
            self.inertia = np.array([mass, mass, mass, rotational_inertia[0], rotational_inertia[1], rotational_inertia[2]])

        self.running = False

        self.param_set_client = self.create_client(SetParameters, "controller/set_parameters")
        self.param_set_client.wait_for_service()

        self._action_server = ActionServer(
            self,
            CalibrateDrag,
            'calibrate_drag',
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
        return CancelResponse.REJECT


    ##############################
    # Message Wait Functions
    ##############################

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

    def requested_accel_cb(self, msg: JointState) -> None:
        if self.requested_accel_queue.full():
            self.requested_accel_queue.get_nowait()

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

    def wait_for_requested_accel_msg(self):
        # Since the queue size is 1, if it has stuff in it just read to clear
        if not self.requested_accel_queue.empty():
            self.requested_accel_queue.get_nowait()
            assert self.requested_accel_queue.empty()
        
        return self.requested_accel_queue.get(True)
        
        
    ##############################
    # Parameter Utility Functions
    ##############################

    def update_controller_config(self, config: dict):
        parameters = []
        for entry in config:
            param = Parameter()
            param.name = entry
            param_value = ParameterValue()
            if type(config[entry]) == list:
                param_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param_value.double_array_value = config[entry]
            else:
                param_value.type = ParameterType.PARAMETER_DOUBLE
                param_value.double_value = float(config[entry])
            param.value = param_value
            parameters.append(param)
        
        request = SetParameters.Request()
        request.parameters = parameters
        
        future = self.param_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: SetParameters.Response = future.result

        
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


    # Example use: r, p, y = get_euler(odom)
    # RPY in radians
    def get_euler(self, odom_msg):
        quat = odom_msg.pose.pose.orientation
        quat = [quat.w, quat.x, quat.y, quat.z]
        return euler.quat2euler(quat, 'sxyz')
    
    #computes the distance between two Vector3s.
    def distance(self, v1, v2):
        x = v2.x - v1.x
        y = v2.y - v1.y
        z = v2.z - v1.z
        
        return sqrt(x * x + y * y + z * z)

    def restrict_angle(self, angle):
        return ((angle + 180) % 360 ) - 180

    # Roll, Pitch, and Yaw configuration
    def to_orientation(self, r, p, y):
        r *= pi / 180
        p *= pi / 180
        y *= pi / 180
        w,x,y,z = euler.euler2quat(r, p, y, axes='sxyz')

        angularCommand = ControllerCommand(mode=ControllerCommand.POSITION, setpoint_quat=Quaternion(w=w, x=x, y=y, z=z))
        self.angular_pub.publish(angularCommand)
        
        time.sleep(5)
        
        
    def to_position(self, x, y, z):
        linear_command = ControllerCommand(mode=ControllerCommand.POSITION, setpoint_vect=Vector3(x=x, y=y, z=z))
        self.linear_pub.publish(linear_command)
        
        startTime = self.get_clock().now()
        
        #block until either position is acheived or 30 seconds elapses
        while(self.distance(linear_command.setpoint_vect , self.wait_for_odometry_msg().pose.pose.position) > 0.2 and (self.get_clock().now() - startTime).nanoseconds < 3e10):            
            time.sleep(1)
            
    def correct_depth(self, z):
        odom = self.wait_for_odometry_msg()
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        
        #move robot to current xy position at desired depth
        self.to_position(x=x, y=y, z=z)
        

    # Apply force on corresponding axes and record velocities
    def collect_data(self, axis, velocity):
        publish_velocity = [
            lambda x: self.linear_pub.publish(ControllerCommand(mode=ControllerCommand.VELOCITY, setpoint_vect=Vector3(x=x,   y=0.0, z=0.0))),
            lambda y: self.linear_pub.publish(ControllerCommand(mode=ControllerCommand.VELOCITY, setpoint_vect=Vector3(x=0.0, y=y,   z=0.0))),
            lambda z: self.linear_pub.publish(ControllerCommand(mode=ControllerCommand.VELOCITY, setpoint_vect=Vector3(x=0.0, y=0.0, z=z  ))),
            lambda x: self.angular_pub.publish(ControllerCommand(mode=ControllerCommand.VELOCITY, setpoint_vect=Vector3(x=x,   y=0.0, z=0.0))),
            lambda y: self.angular_pub.publish(ControllerCommand(mode=ControllerCommand.VELOCITY, setpoint_vect=Vector3(x=0.0, y=y,   z=0.0))),
            lambda z: self.angular_pub.publish(ControllerCommand(mode=ControllerCommand.VELOCITY, setpoint_vect=Vector3(x=0.0, y=0.0, z=z  )))
        ]

        get_twist = [
            lambda odom: odom.twist.twist.linear.x,
            lambda odom: odom.twist.twist.linear.y,
            lambda odom: odom.twist.twist.linear.z,
            lambda odom: odom.twist.twist.angular.x,
            lambda odom: odom.twist.twist.angular.y,
            lambda odom: odom.twist.twist.angular.z
        ]

        get_accel = [
            lambda twist: twist.linear.x,
            lambda twist: twist.linear.y,
            lambda twist: twist.linear.z,
            lambda twist: twist.angular.x,
            lambda twist: twist.angular.y,
            lambda twist: twist.angular.z
        ]

        
        publish_velocity[axis](float(velocity))

        time.sleep(1)
        last_vel = 0
        stable_measurements = 0
        while stable_measurements < 10:
            odom_msg = self.wait_for_odometry_msg()
            cur_vel = get_twist[axis](odom_msg)

            if abs((cur_vel - last_vel) / cur_vel) >= 0.1:
                stable_measurements = 0
            else:
                stable_measurements += 1
            last_vel = cur_vel
            time.sleep(0.1)

        velocities = []
        for _ in range (10):
            odom_msg = self.wait_for_odometry_msg()
            velocities.append(get_twist[axis](odom_msg))
            time.sleep(0.1)

        forces = []
        for _ in range (10):
            accel_msg = self.wait_for_requested_accel_msg()
            forces.append(get_accel[axis](accel_msg) * self.inertia[axis])
            time.sleep(0.1)

        publish_velocity[axis](0.0)
        time.sleep(0.1)

        return np.average(velocities), -np.average(forces)
    
    # Calcualte the multivariable linear regression of linear and quadratic damping
    def calculate_parameters(self, velocities, forces):
        y = np.array(forces)
        X = np.array([velocities, np.abs(velocities) * np.array(velocities)])
        X = X.T # transpose so input vectors are along the rows
        return np.linalg.lstsq(X,y)[0]

    # Set depth in rqt before running action
    def execute_cb(self, goal_handle: ServerGoalHandle):
        self.running = True

        self.update_controller_config({
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

        # Initialize starting position of robot
        odom_msg = self.wait_for_odometry_msg()
        startYaw = self.get_euler(odom_msg)[2] * 180 / pi
        startPosition = odom_msg.pose.pose.position
        self.get_logger().info("Starting drag calibration")

        linear_params = np.zeros(6)
        quadratic_params = np.zeros(6)

        # Euler for ease of use
        axes_test_orientations = [
            [0, 0, startYaw],
            [0, 0, self.restrict_angle(startYaw - 90)],
            [0, -85, startYaw],
            [0, -85, startYaw],
            [90, 0, startYaw],
            [0, 0, startYaw]
        ]

        axis_velocities = [
            [0.05, -0.05, .1, -.1, .2, -.2],
            [0.05, -0.05, .1, -.1, .2, -.2],
            [-0.05, 0.05, -.1, .1, -.2, .2],
            [0.2, -0.2, 0.5, -0.5, 1.2, -1.2],
            [0.2, -0.2, 0.5, -0.5, 1.2, -1.2],
            [0.2, -0.2, 0.5, -0.5, 1.2, -1.2],
        ]

        for axis in range(6):
            self.to_position(x=startPosition.x, y=startPosition.y, z=startPosition.z)
            self.to_orientation(*axes_test_orientations[axis])

            forces = []
            velocities = []
            for requested_velocity in axis_velocities[axis]:
                velocity, force = self.collect_data(axis, requested_velocity)
                forces.append(force)
                velocities.append(velocity)
                
                #move robot to correct depth
                self.correct_depth(startPosition.z)

            linear_params[axis], quadratic_params[axis] = self.calculate_parameters(velocities, forces)
            self.get_logger().info("Linear: %f" % linear_params[axis])
            self.get_logger().info("Quadratic: %f" % quadratic_params[axis])

        self.to_orientation(0, 0, startYaw)

        self.get_logger().info("Drag calibration completed. New calibration values applied")

        self.update_controller_config({
            "linear_damping_x": linear_params[0],
            "linear_damping_y": linear_params[1],
            "linear_damping_z": linear_params[2],
            "linear_damping_rot_x": linear_params[3],
            "linear_damping_rot_y": linear_params[4],
            "linear_damping_rot_z": linear_params[5],
            "quadratic_damping_x": quadratic_params[0],
            "quadratic_damping_y": quadratic_params[1],
            "quadratic_damping_z": quadratic_params[2],
            "quadratic_damping_rot_x": quadratic_params[3],
            "quadratic_damping_rot_y": quadratic_params[4],
            "quadratic_damping_rot_z": quadratic_params[5]
        })

        self._result.linear_drag = list(linear_params)
        self._result.quadratic_drag = list(quadratic_params)

        self.running = False
        goal_handle.succeed()
        return self._result

def main(args=None):
    rclpy.init(args=args)

    thruster_test_action_server = CalibrateDragActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_test_action_server, executor=executor)

    thruster_test_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
