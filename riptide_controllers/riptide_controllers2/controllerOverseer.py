import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default, qos_profile_system_default

from ros2node.api import get_node_names, NodeName

from tf2_ros.transform_listener import TransformListener
from tf2_ros import Buffer

from tf_transformations import euler_matrix

import numpy as np

import yaml

from riptide_msgs2.msg import DshotPartialTelemetry, KillSwitchReport, DshotCommand
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int16, Int32MultiArray, Float32MultiArray

ERROR_PATIENCE = 1.0

PARAMETER_SCALE = 1000000

THRUSTER_SOLVER_NODE_NAME = "ThrusterSolver"

THRUSTER_SOLVER_WRENCH_MATRIX_PARAM = "talos_wrenchmat"
THRUSTER_SOLVER_WEIGHT_MATRIX_PARAM = "talos_thruster_weights"
THRUSTER_SOLVER_SYSTEM_LIMIT_PARAM = "talos_sys_lim"
THRUSTER_SOLVER_INDIVIDUAL_LIMIT_PARAM = "talos_indiv_lim"
THRUSTER_SOLVER_SCALING_PARAM = "thruster_solver_scaling_parameters"
THRUSTER_SOLVER_FORCE_RPM_COEFFICENTS_PARAM = "talos_force_curve_coefficents"

#manages parameters for the controller / thruster solver
class controllerOverseer(Node):

    def __init__(self):
        super().__init__("controllerOverseer")

        #get robot name 
        self.declare_parameter("robot", "")
        self.robotName = self.get_parameter("robot").value

        #get robot config file path
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if(config_path == ''):
            #remove before running on robot
            config_path = "src/riptide_core/riptide_descriptions/config/talos.yaml"

            #self.get_logger().fatal("Controller Overseer: cannot find vehicle configuration file!")

        #open configuration file
        with open(config_path, "r") as config:
            config_file = yaml.safe_load(config)

        #read in thruster pose data
        thruster_info = config_file['thrusters']

        #read in com data
        com = config_file["com"]

        #generate the thruster effect matrix
        self.thrusterEffects = np.zeros(shape=(8,6))
        for i, thruster in enumerate(thruster_info):
            pose = np.array(thruster["pose"])

            #calculate the force vector generated by the thruster
            forceVector = np.matmul(euler_matrix(pose[3], pose[4], pose[5])[:3, :3], np.array([1,0,0]))
            positionFromCOM = pose[:3] - com
            torque = np.cross(positionFromCOM[:3], forceVector)

            #insert into thruster effect matrix
            self.thrusterEffects[i] = [forceVector[0], forceVector[1], forceVector[2], torque[0], torque[1], torque[2]]

        #read in coefficents
        thruster_solver_info = config_file["thruster_solver"]
        self.forceToRPMCoefficents = thruster_solver_info["force_to_rpm_coefficents"]

        #default thruster weights and working thrusters
        self.activeThrusters = [True, True, True, True, True, True, True, True]
        self.submerdgedThrusters = [True, True, True, True, True, True, True, True]
        self.thrusterWeights = [1,1,1,1,1,1,1,1]

        #declare limit params
        self.declare_parameter("System_Thruster_Limit", 20)
        self.declare_parameter("Individual_Thruster_Limit", 8)

        # the thruster mode
        self.thrusterMode = 1

        #declare pubs and subs

        #thruster telemetry
        self.create_subscription(DshotPartialTelemetry, "state/thrusters/telemetry", self.thrusterTelemetryCB, qos_profile_system_default)
        #thruster Weights
        self.setThrusterSolverParamsClient = self.create_client(SetParameters, f"/{THRUSTER_SOLVER_NODE_NAME}/set_parameters")
        #thruster mode
        self.create_subscription(Int16, "thrusterSolver/thrusterState", self.setThrusterModeCB, qos_profile_system_default)
        #software kill 
        self.killPub = self.create_publisher(KillSwitchReport, "command/software_kill", qos_profile_system_default)
        #odometry filtered
        self.create_subscription(Odometry, "odometry/filtered", self.odometryCB, qos_profile_system_default)

        #republish matlab to firmware
        #TODO remove
        self.create_subscription(Int32MultiArray, "reqforce", self.forceRepublishCb, qos_profile_system_default)
        self.create_subscription(Int32MultiArray, "reqRPM", self.rpmRepublishCb, qos_profile_system_default)

        self.rpmCommandPub = self.create_publisher(DshotCommand, "/talos/command/thruster_rpm", qos_profile_system_default)
        self.forceCommandPub = self.create_publisher(Float32MultiArray, "/talos/thruster_forces", qos_profile_system_default)

        #declare transform 
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        #the tf namespace
        self.tfNamespace = self.get_parameter("robot").value

        #the plane at which to kill power to thrusters
        self.declare_parameter("thrusterKillPlace", 0)

        #start time
        self.startTime = None

        #is the thrusterSolver active
        self.solverActive = False

        #attempt to set the wrench matrix in the thruster solver
        self.create_timer(1.0, self.checkIfSolverActive)


    def thrusterTelemetryCB(self, msg: DshotPartialTelemetry):
        #wether or not the thrusterSolverweigths need adjusted
        adjustWeights = False

        # which thrusters - groups of 4
        if(msg.start_thruster_num == 0):
            #check wether each thruster is active
            for i, esc in enumerate(msg.esc_telemetry):
                if not esc.present:
                    if self.activeThrusters[i] == True:
                        #if change, adjust the thruster weights
                        self.activeThrusters[i] = False
                        adjustWeights = True
                else:
                    if self.activeThrusters[i] == False:
                        self.activeThrusters[i] = True
                        adjustWeights = True
        else:
            for i, esc in enumerate(msg.esc_telemetry):
                if not esc.present:
                    if self.activeThrusters[i + 4] == True:
                        self.activeThrusters[i + 4] = False
                        adjustWeights = True
                else:
                    if self.activeThrusters[i + 4] == False:
                        self.activeThrusters[i + 4] = True
                        adjustWeights = True

        #adjust the weights of the thruster solver if anything has changed
        if(adjustWeights):
            self.adjustThrusterWeights()

    def setThrusterModeCB(self, msg:Int16):
        #change the thruster mode

        #Modes -----
        #   1 - Normal
        #   2 - Low Downdraft

        if not (msg.data == self.thrusterMode):
            self.thrusterMode = msg.data

            #update the weights
            self.adjustThrusterWeights()
        
    def odometryCB(self, msg):
        #check if thrusters are submerged - everytime odom is updated - just using as a frequency 

        #start the start time on first cb
        if(self.startTime == None):
            self.startTime = self.get_clock().now()

        submerged = {False, False, False, False, False, False, False, False}

        try:
            #look at each thruster
            for i, thursterSufaced in enumerate(self.submerdgedThrusters):
                pos = self.tfBuffer.lookup_transform(f"{self.tfNamespace}/thruster_{i}", "world", self.get_clock().now())

                #if thruster is above the kill plane
                if pos.transform.translation.z < self.get_parameter("thrusterKillPlane"):
                    submerged[i] = True
                
            if not (np.array_equal(submerged, self.submerdgedThrusters)):
                #if a different thruster combo is submerdged, adjust, the weights
                self.submerdgedThrusters = submerged
                self.adjustThrusterWeights()
                
        except Exception as ex:
            if(self.get_clock().now() >= ERROR_PATIENCE + self.startTime):
                self.get_logger().error("Thruster Position Lookup failed with exception: " + ex)
        
    def adjustThrusterWeights(self):
        #TODO add weight values into descriptions

        #number of active thrusters 
        activeThrusterCount = 0
        submerdgedThrusters = 0

        #shutoff inactive thrusters - disabled or broken
        for i, isActive in self.activeThrusters:
            if(isActive):
                activeThrusterCount += 1

                #play around with weights for thrusters above surface
                if not (self.submerdgedThrusters[i] == True):
                    #if thruster is not submerdged
                    self.thrusterWeights[i] = 4
                else:
                    #thruster is active and submerdged
                    submerdgedThrusters += 1
                    self.thrusterWeights = 1

            else:
                #if a thruster is inactive - raise the cost of "using" thruster
                self.thrusterWeights[i] = 999999
        
        if(activeThrusterCount <= 6):
            #if system is not full actuated, it cannot be optimized, very high rpms / force can be requested
            #for the safety of the system, this will autodisable robot (probably)
            #remove if PIA

            self.get_logger().error("System has become underactuated. Only:  " + str(activeThrusterCount) + " thrusters are active. Killing Thrusters!")
            
            #add a kill switch report
            msg = KillSwitchReport()
            msg.sender_id = self.get_name()
            msg.kill_switch_id = 3 # this is the debug switch - ask firmware to fix
            msg.switch_asserting_kill = True
            msg.switch_needs_update = True

            self.killPub.publish(msg)
            
        if(submerdgedThrusters >= 8):
            #take into account control modes only if all actuators are working
            
            if(self.thrusterMode == 2):
                #apply low downdraft
                self.thrusterWeights[4] = 3
                self.thrusterWeights[5] = 3
                
        #scale and round all weights
        weights = []
        for weight in self.thrusterWeights:
            weights.append(int(weight * PARAMETER_SCALE))

        # #declare the weight paramter value
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = weights

        param = Parameter()
        param.value = val
        param.name = THRUSTER_SOLVER_WEIGHT_MATRIX_PARAM

        #setup service request
        request = SetParameters.Request()
        request.parameters = [param]

        self.future = self.setThrusterSolverParamsClient.call_async(request)

    def checkIfSolverActive(self):
        #attempt to set the wrench matrix in the simulink node

        foundSolver = False
        #see if the matlab node is running
        for nodeName in get_node_names(node=self):
            if nodeName.name == THRUSTER_SOLVER_NODE_NAME:
                foundSolver = True

        if(self.solverActive != foundSolver):
            if(foundSolver):
                #thruster solver came online
                self.paramTimer = self.create_timer(1.0, self.setSolverParams)
            else:
                #lost thruster solver
                self.get_logger().error("Lost Thruster Solver!")


        #update wether or not the solver has been found
        self.solverActive = foundSolver

    def setSolverParams(self):            
        #make the thruster effect matrix 1-D
        effects = []
        for thruster in self.thrusterEffects:
            for effect in thruster:
                effects.append(int(effect * PARAMETER_SCALE))

        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = effects

        param = Parameter()
        param.value = val
        param.name = THRUSTER_SOLVER_WRENCH_MATRIX_PARAM

        val1 = ParameterValue()
        val1.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val1.integer_array_value = self.thrusterWeights

        param1 = Parameter()
        param1.value = val1
        param1.name = THRUSTER_SOLVER_WEIGHT_MATRIX_PARAM

        val2 = ParameterValue()
        val2.type = ParameterType.PARAMETER_INTEGER
        val2.integer_value = int(self.get_parameter("System_Thruster_Limit").value)

        param2 = Parameter()
        param2.value = val2
        param2.name = THRUSTER_SOLVER_SYSTEM_LIMIT_PARAM

        val3 = ParameterValue()
        val3.type = ParameterType.PARAMETER_INTEGER
        val3.integer_value = int(self.get_parameter("Individual_Thruster_Limit").value)

        param3 = Parameter()
        param3.value = val3
        param3.name = THRUSTER_SOLVER_INDIVIDUAL_LIMIT_PARAM

        val4 = ParameterValue()
        val4.type = ParameterType.PARAMETER_INTEGER
        val4.integer_value = PARAMETER_SCALE

        param4 = Parameter()
        param4.value = val4
        param4.name = THRUSTER_SOLVER_SCALING_PARAM

        #scale and round the coefficents
        coefficents = []
        for coefficent in self.forceToRPMCoefficents:
            coefficents.append(int(coefficent * PARAMETER_SCALE))

        val5 = ParameterValue()
        val5.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val5.integer_array_value = coefficents

        param5 = Parameter()
        param5.value = val5
        param5.name = THRUSTER_SOLVER_FORCE_RPM_COEFFICENTS_PARAM

        #setup service request
        request = SetParameters.Request()
        request.parameters = [param, param1, param2, param3, param4, param5]

        self.future = self.setThrusterSolverParamsClient.call_async(request)

        #cancel the timer
        self.paramTimer.cancel()

    #remove once matlab publishes to correct message
    def forceRepublishCb(self, msg:Int32MultiArray):
        #the message to republish to
        pub_msg = Float32MultiArray()
                
        if(len(msg.data) == 8):
            for datum in msg.data:
                #scale by .000001
                pub_msg.data.append(datum * .000001)


            #publish the message
            self.forceCommandPub.publish(pub_msg)
        else:
            self.get_logger().warn("Forces form solver has wrong length of: " + str(len(msg.data)))

    #remove once matlab publishes to correct message
    def rpmRepublishCb(self, msg:Int32MultiArray):
        #the message to republish to
        pub_msg = DshotCommand()
        
        if(len(msg.data) == 8):
            for i, datum in enumerate(msg.data):
                #scale by .000001
                pub_msg.values[i] = (datum)

            #publish the message
            self.rpmCommandPub.publish(pub_msg)
        else:
            self.get_logger().warn("RPM form solver has wrong length of: " + str(len(msg.data)))

def main(args=None):
    rclpy.init(args=args)

    co = controllerOverseer()

    rclpy.spin(co)

if __name__ == '__main__':
    main()

            
