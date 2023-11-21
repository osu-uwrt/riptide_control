#! /usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default, qos_profile_system_default

from ament_index_python import get_package_share_directory

from ros2node.api import get_node_names, NodeName

from tf2_ros.transform_listener import TransformListener
from tf2_ros import Buffer

from tf_transformations import euler_matrix

import numpy as np

import yaml

from riptide_msgs2.msg import DshotPartialTelemetry, DshotCommand
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int16, Int32MultiArray, Float32MultiArray, Empty
from geometry_msgs.msg import Twist
from rclpy.time import Time

ERROR_PATIENCE = 1.0

PARAMETER_SCALE = 1000000

#
# THRUSTER SOLVER PARAM NAMES
#
THRUSTER_SOLVER_WRENCH_MATRIX_PARAM = "talos_wrenchmat"
THRUSTER_SOLVER_WEIGHT_MATRIX_TOPIC = "controller/solver_weights"
THRUSTER_SOLVER_SYSTEM_LIMIT_PARAM = "talos_sys_lim"
THRUSTER_SOLVER_INDIVIDUAL_LIMIT_PARAM = "talos_indiv_lim"
THRUSTER_SOLVER_SCALING_PARAM = "thruster_solver_scaling_parameters"
THRUSTER_SOLVER_FORCE_RPM_COEFFICENTS_PARAM = "talos_force_curve_coefficents"
THRUSTER_SOLVER_DISABLE_WEIGHTS = "thruster_solver_disable_weight"

#
# SMC PARAM NAMES
# 
ACTIVE_CONTROLLER_DRAG_COEFFICENTS_PARAM = "talos_drag_coefficents"
ACTIVE_CONTROLLER_ANGULAR_REGEN_PARAM = "talos_angular_regeneration_threshhold"
ACTIVE_CONTROLLER_LINEAR_REGEN_PARAM = "talos_linear_regeneration_threshhold"
ACTIVE_CONTROLLER_ANGULAR_STATIONKEEP_PARAM = "talos_angular_stationkeep_threshold"
ACTIVE_CONTROLLER_LINEAR_STATIONKEEP_PARAM = "talos_linear_stationkeep_threshold"
ACTIVE_CONTROLLER_MASS_PARAM = "talos_mass"
ACTIVE_CONTROLLER_ROTATIONAL_INERTIAS_PARAM = "talos_rotational_inertias"
ACTIVE_CONTROLLER_LAMBDA_PARAM = "talos_lambda"
ACTIVE_CONTROLLER_ETA_0_PARAM = "talos_Eta_Order_0"
ACTIVE_CONTROLLER_ETA_1_PARAM = "talos_Eta_Order_1"
ACTIVE_CONTROLLER_VMAX_PARAM = "talos_vMax"
ACTIVE_CONTROLLER_AMAX_PARAM = "talos_aMax"
ACTIVE_CONTROLLER_JAMX_PARAM = "talos_jMax"
ACTIVE_CONTROLLER_RADIAL_FUNC_COUNT_PARAM = "talos_radial_function_count"
ACTIVE_CONTROLLER_SCALING_PARAM = "parameter_scaling_factor"

#
# PID PARAM NAMES
#
ACTIVE_CONTROLLER_PID_P_GAINS_PARAM = "pid_PGains"
ACTIVE_CONTROLLER_PID_I_GAINS_PARAM = "pid_IGains"
ACTIVE_CONTROLLER_PID_D_GAINS_PARAM = "pid_DGains"
ACTIVE_CONTROLLER_PID_RESET_THRESHOLD_PARAM = "pid_reset_threshold"
ACTIVE_CONTROLLER_PID_SCALING_FACTOR_PARAM = "pid_scaling_parameters"

FF_PUBLISH_PARAM = "disable_native_ff"
FF_TOPIC_NAME = "controller/FF_body_force"

RPM_PUBLISH_PERIOD = .02
WEIGHTS_FORCE_UPDATE_PERIOD = 1

G = 9.8067

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

        #get thruster solver node name
        self.declare_parameter("thruster_solver_node_name", "")
        self.thrusterSolverName = self.get_parameter("thruster_solver_node_name").value

        #get thruster solver node name
        self.declare_parameter("active_controller_node_name", "")
        self.activeControllerName = self.get_parameter("active_controller_node_name").value

        self.declare_parameter(FF_PUBLISH_PARAM, False)

        #read in yaml
        self.readInRobotYaml()

        #generate thruster force matrix
        self.genertateThrusterForceMatrix(self.thruster_info, self.com)

        #default thruster weights and working thrusters
        self.activeThrusters = [True, True, True, True, True, True, True, True]
        self.submerdgedThrusters = [True, True, True, True, True, True, True, True]
        self.thrusterWeights = [1,1,1,1,1,1,1,1]

        # the thruster mode
        self.thrusterMode = 1

        #declare pubs and subs

        #thruster telemetry
        self.create_subscription(DshotPartialTelemetry, "state/thrusters/telemetry", self.thrusterTelemetryCB, qos_profile_system_default)

        #thruster solver parameters
        self.setThrusterSolverParamsClient = self.create_client(SetParameters, f"{self.thrusterSolverName}/set_parameters")

        #active controller parameters
        self.setActiveControllerParamsClient = self.create_client(SetParameters, f"{self.activeControllerName}/set_parameters")

        #thruster mode
        self.create_subscription(Int16, "thrusterSolver/thrusterState", self.setThrusterModeCB, qos_profile_system_default)

        #odometry filtered
        self.create_subscription(Odometry, "odometry/filtered", self.odometryCB, qos_profile_system_default)

        #republish matlab to firmware
        #TODO remove
        self.create_subscription(Int32MultiArray, "reqforce", self.forceRepublishCb, 10)
        self.create_subscription(Int32MultiArray, "reqRPM", self.rpmRepublishCb, qos_profile_system_default)

        self.rpmCommandPub = self.create_publisher(DshotCommand, "command/thruster_rpm", qos_profile_system_default)
        self.forceCommandPub = self.create_publisher(Float32MultiArray, "thruster_forces", qos_profile_system_default)

        #pub for thruster weights
        self.weightsPub = self.create_publisher(Int32MultiArray, THRUSTER_SOLVER_WEIGHT_MATRIX_TOPIC, qos_profile_system_default)

        #pub ff force
        self.ffPub = self.create_publisher(Twist, FF_TOPIC_NAME, qos_profile_system_default)

        #sub to republish params trigger
        self.repubSubActive = self.create_subscription(Empty, "controller_overseer/update_active_params", self.repubActiveParamsCb, qos_profile_system_default)
        self.repubSubThruster = self.create_subscription(Empty, "controller_overseer/update_ts_params", self.repubThrusterSolverParamsCb, qos_profile_system_default)

        #declare transform
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        #the tf namespace
        # TODO: FIX
        # self.tfNamespace = self.get_parameter("robot").value
        self.tfNamespace = "talos"

        #start time
        self.startTime = None

        #is the thrusterSolver active
        self.solverActive = False

        #is the SMC controller active
        self.activeActive = False

        #attempt to set the wrench matrix in the thruster solver
        self.create_timer(1.0, self.checkIfSimulinkActive)

        #publsh msgs at frequency
        self.pubRPMMsg = None
        self.pubForceMsg = None

        #start the publish loop
        self.create_timer(RPM_PUBLISH_PERIOD, self.pubCB)
        self.create_timer(WEIGHTS_FORCE_UPDATE_PERIOD, self.adjustThrusterWeights)

        self.pubTimer = None
        self.enabled = True
        self.publishingFF = True

    def genertateThrusterForceMatrix(self, thruster_info, com):
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

    def readInRobotYaml(self):
        #reload in file from scratch just incase the target yaml has changed
        configPath = self.get_parameter("vehicle_config").value
        if(configPath == ''):
            # Try to locate the source directory, and use that configuration file directly
            descriptions_share_dir = get_package_share_directory("riptide_descriptions2")

            robot_config_subpath = os.path.join("config", self.robotName + ".yaml")

            # Set fallback config path if we can't find the one in source
            configPath = os.path.join(descriptions_share_dir, robot_config_subpath)

            # Try to locate the share directory
            dir_split = os.path.normpath(descriptions_share_dir).split(os.sep)
            if 'install' in dir_split:
                colcon_root = '/'.join(dir_split[:dir_split.index('install')])
                colcon_source_dir = os.path.join(colcon_root, "src")

                # List of possible paths we should check for a config file
                possible_paths = [
                    os.path.join(colcon_source_dir, "riptide_core", "riptide_descriptions", robot_config_subpath),
                    os.path.join(colcon_source_dir, "riptide_descriptions", robot_config_subpath),
                ]

                for path_check in possible_paths:
                    if os.path.exists(path_check):
                        configPath = path_check
                        self.get_logger().info(f"Discovered source directory, overidding descriptions to use '{configPath}'")
                        break

        with open(configPath, "r") as config:
            config_file = yaml.safe_load(config)

        #read in kill plane height
        self.killPlane = config_file["controller_overseer"]["thruster_kill_plane"]

        #read in thruster pose data
        self.thruster_info = config_file['thrusters']

        #read in com data
        self.com = config_file["com"]

        #read in coefficents
        thruster_solver_info = config_file["thruster_solver"]
        self.forceToRPMCoefficents = thruster_solver_info["force_to_rpm_coefficents"]

        #read in weight info
        self.defaultWeight = thruster_solver_info["default_weight"]
        self.surfaceWeight = thruster_solver_info["surfaced_weight"]
        self.disabledWeight = thruster_solver_info["disable_weight"]
        self.lowDowndraftWeight = thruster_solver_info["low_downdraft_weight"]

        #read in solver limit params
        self.systemThrustLimit = thruster_solver_info["system_thrust_limit"]
        self.individualThrustLimit = thruster_solver_info["individual_thrust_limit"]

        #read in ff properties
        self.talos_mass = config_file["mass"]
        self.talos_COM = config_file["com"]
        self.talos_base_wrench = config_file["controller"]["feed_forward"]["base_wrench"]
        self.talos_rotational_inertias = config_file["inertia"]

        #SMC specific parameters
        self.talos_drag_coefficents = config_file["controller"]["SMC"]["damping"]
        self.talos_angular_regen = config_file["controller"]["SMC"]["SMC_params"]["angular_regeneration_threshold"]
        self.talos_linear_regen = config_file["controller"]["SMC"]["SMC_params"]["linear_regeneration_threshold"]
        self.talos_angular_stationkeep = config_file["controller"]["SMC"]["SMC_params"]["angular_stationkeep_threshold"]
        self.talos_linear_stationkeep = config_file["controller"]["SMC"]["SMC_params"]["linear_stationkeep_threshold"]
        self.talos_eta_0_values = config_file["controller"]["SMC"]["SMC_params"]["eta_order_0"]
        self.talos_eta_1_values = config_file["controller"]["SMC"]["SMC_params"]["eta_order_1"]
        self.talos_lambda_values = config_file["controller"]["SMC"]["SMC_params"]["lambda"]

        #motion generation params
        self.talos_vmax_values = config_file["controller"]["SMC"]["motion_profile_params"]["max_velocity"]
        self.talos_amax_values = config_file["controller"]["SMC"]["motion_profile_params"]["max_acceleration"]
        self.talos_jmax_values = config_file["controller"]["SMC"]["motion_profile_params"]["max_jerk"]
        self.talos_radial_function_count = config_file["controller"]["SMC"]["motion_profile_params"]["radial_function_count"]
        
        #PID specific parameters
        self.talos_p_gains = config_file["controller"]["PID"]["p_gains"]
        self.talos_i_gains = config_file["controller"]["PID"]["i_gains"]
        self.talos_d_gains = config_file["controller"]["PID"]["d_gains"]
        self.talos_pid_reset_threshold = config_file["controller"]["PID"]["reset_threshold"]


    def thrusterTelemetryCB(self, msg: DshotPartialTelemetry):
        #wether or not the thrusterSolverweigths need adjusted
        adjustWeights = False

        # which thrusters - groups of 4
        if(msg.start_thruster_num == 0):
            #check wether each thruster is active
            for i, esc in enumerate(msg.esc_telemetry):
                if not esc.thruster_ready:
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
                if not esc.thruster_ready:
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
        if(self.startTime is None):
            self.startTime = self.get_clock().now()

        submerged = [False, False, False, False, False, False, False, False]

        try:
            #look at each thruster
            for i, thursterSufaced in enumerate(self.submerdgedThrusters):
                pos = self.tfBuffer.lookup_transform("world", f"{self.tfNamespace}/thruster_{i}", Time())

                #if thruster is above the kill plane
                if pos.transform.translation.z < self.killPlane:
                    submerged[i] = True

            if not (np.array_equal(submerged, self.submerdgedThrusters)):
                #if a different thruster combo is submerdged, adjust, the weights
                self.submerdgedThrusters = submerged
                self.adjustThrusterWeights()

        except Exception as ex:
            if(self.get_clock().now().to_msg().sec >= ERROR_PATIENCE + self.startTime.to_msg().sec):
                self.get_logger().error("Thruster Position Lookup failed with exception: " + str(ex))

    def adjustThrusterWeights(self):
        #TODO add weight values into descriptions

        #number of active thrusters
        activeThrusterCount = 0
        submerdgedThrusters = 0

        #shutoff inactive thrusters - disabled or broken
        for i, isActive in enumerate(self.activeThrusters):
            if(isActive):
                activeThrusterCount += 1

                #play around with weights for thrusters above surface
                if not (self.submerdgedThrusters[i] == True):
                    #if thruster is not submerdged
                    self.thrusterWeights[i] = self.surfaceWeight
                else:
                    #thruster is active and submerdged
                    submerdgedThrusters += 1
                    self.thrusterWeights[i] = self.defaultWeight

            else:
                #if a thruster is inactive - raise the cost of "using" thruster
                self.thrusterWeights[i] = self.disabledWeight

        if(activeThrusterCount <= 6):
            #if system is not full actuated, it cannot be optimized, very high rpms / force can be requested
            #for the safety of the system, this will autodisable robot (probably)
            #remove if PIA

            self.get_logger().error("System has become underactuated. Only:  " + str(activeThrusterCount) + " thrusters are active. Killing Thrusters!")
            self.enabled = False
        else:
            self.enabled = True

        if(submerdgedThrusters >= 8):
            #take into account control modes only if all actuators are working

            if(self.thrusterMode == 2):
                #apply low downdraft
                self.thrusterWeights[4] = self.lowDowndraftWeight
                self.thrusterWeights[5] = self.lowDowndraftWeight

        #impose disable weight if nessecary
        if(self.thrusterMode == 0):
            self.thrusterWeights = [0,0,0,0,0,0,0,0]

        msg = Int32MultiArray()

        #scale and round all weights
        weights = []
        for weight in self.thrusterWeights:
            weights.append(int(weight * PARAMETER_SCALE))

        # self.get_logger().info(str(weights))

        msg.data = weights

        #publish weights
        self.weightsPub.publish(msg)

    def checkIfSimulinkActive(self):
        #attempt to set the wrench matrix in the simulink node

        foundSolver = False
        foundActiveControl = False

        thrusterSolverSetParamName = f"{self.thrusterSolverName}/set_parameters"
        activeControllerSetParamName = f"{self.activeControllerName}/set_parameters"

        #see if the matlab node is running
        for nodeName in get_node_names(node=self):

            #check the thruster solver
            if self.thrusterSolverName in nodeName.name:
                foundSolver = True
                thrusterSolverSetParamName = f"{nodeName.full_name}/set_parameters"

            #check the active controller
            if self.activeControllerName in nodeName.name:
                foundActiveControl = True
                activeControllerSetParamName = f"{nodeName.full_name}/set_parameters"


        if(self.solverActive != foundSolver):
            if(foundSolver):
                #thruster solver came online. now that we know its name we can set its parameters
                self.get_logger().info(f"Setting parameters using service {thrusterSolverSetParamName}")
                self.setThrusterSolverParamsClient = self.create_client(SetParameters, thrusterSolverSetParamName)
                self.paramTimerTS = self.create_timer(1.0, self.setSolverParams)

                self.get_logger().info("Found Thruster Solver!")
            else:
                #lost thruster solver
                self.get_logger().error("Lost Thruster Solver!")

                #cancel the set default params operation
                self.paramTimerTS.cancel()

                #cancel the publishing
                self.pubTimer.cancel()

        if(self.activeActive != foundActiveControl):
            if(foundActiveControl):
                #active control has come online
                self.get_logger().info(f"Setting parameters using service {activeControllerSetParamName}")
                self.setActiveControllerParamsClient = self.create_client(SetParameters, activeControllerSetParamName)
                self.paramTimerActive = self.create_timer(1.0, self.setActiveParams)

                self.get_logger().info("Found Active Controller!")
            else:
                #lost thruster solver
                self.get_logger().error("Lost Active Controller!")

                #cancel the set default params operation
                self.paramTimerActive.cancel()


        if not (self.get_parameter(FF_PUBLISH_PARAM).value):
            #publish the ff

            self.publishingFF = True

            msg = Twist()
            msg.linear.x = self.talos_base_wrench[0]
            msg.linear.y = self.talos_base_wrench[1]
            msg.linear.z = self.talos_base_wrench[2]
            msg.angular.x = self.talos_base_wrench[3]
            msg.angular.y = self.talos_base_wrench[4]
            msg.angular.z = self.talos_base_wrench[5]

            self.ffPub.publish(msg)
        elif(self.publishingFF == True):
            #send zero before finishing
            self.publishingFF = False

            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            self.ffPub.publish(msg)


        #update wether or not the solver and active control has been found
        self.solverActive = foundSolver
        self.activeActive = foundActiveControl

    def setSolverParams(self):
        self.get_logger().info("Setting Thruster Solver Default Parameters")

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
        val1.type = ParameterType.PARAMETER_INTEGER
        val1.integer_value = int(self.disabledWeight)

        param1 = Parameter()
        param1.value = val1
        param1.name = THRUSTER_SOLVER_DISABLE_WEIGHTS

        val2 = ParameterValue()
        val2.type = ParameterType.PARAMETER_INTEGER
        val2.integer_value = int(self.systemThrustLimit)

        param2 = Parameter()
        param2.value = val2
        param2.name = THRUSTER_SOLVER_SYSTEM_LIMIT_PARAM

        val3 = ParameterValue()
        val3.type = ParameterType.PARAMETER_INTEGER
        val3.integer_value = int(self.individualThrustLimit)

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
        self.paramTimerTS.cancel()
        #ensure the pub timer is running if both models are active
        if(self.solverActive):
            if(self.pubTimer is None):
                self.pubTimer = self.create_timer(RPM_PUBLISH_PERIOD, self.pubCB)
            elif(self.pubTimer.is_canceled()):
                self.pubTimer = self.create_timer(RPM_PUBLISH_PERIOD, self.pubCB)


    def getSMCSetParamRequest(self):
                #drag coefficents
        int_values = []
        for value in self.talos_drag_coefficents:
            int_values.append(int(value * PARAMETER_SCALE))

        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param1 = Parameter()
        param1.value = val
        param1.name = ACTIVE_CONTROLLER_DRAG_COEFFICENTS_PARAM

        #angular regen
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_angular_regen * PARAMETER_SCALE)

        param2 = Parameter()
        param2.value = val
        param2.name = ACTIVE_CONTROLLER_ANGULAR_REGEN_PARAM

        #linear regen
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_linear_regen * PARAMETER_SCALE)

        param3 = Parameter()
        param3.value = val
        param3.name = ACTIVE_CONTROLLER_LINEAR_REGEN_PARAM
        
        #angular stationkeep
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_angular_stationkeep * PARAMETER_SCALE)

        param4 = Parameter()
        param4.value = val
        param4.name = ACTIVE_CONTROLLER_ANGULAR_STATIONKEEP_PARAM

        #linear stationkeep
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_linear_stationkeep * PARAMETER_SCALE)

        param5 = Parameter()
        param5.value = val
        param5.name = ACTIVE_CONTROLLER_LINEAR_STATIONKEEP_PARAM

        #scale and int
        int_values = []
        for value in self.talos_eta_0_values:
            int_values.append(int(value * PARAMETER_SCALE))

        #SMC ETA 0
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param6 = Parameter()
        param6.value = val
        param6.name = ACTIVE_CONTROLLER_ETA_0_PARAM

        #scale and int
        int_values = []
        for value in self.talos_eta_1_values:
            int_values.append(int(value * PARAMETER_SCALE))

        #SMC ETA 0
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param7 = Parameter()
        param7.value = val
        param7.name = ACTIVE_CONTROLLER_ETA_1_PARAM

        #scale and int
        int_values = []
        for value in self.talos_lambda_values:
            int_values.append(int(value * PARAMETER_SCALE))

        #SMC lambda
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param8 = Parameter()
        param8.value = val
        param8.name = ACTIVE_CONTROLLER_LAMBDA_PARAM

        #mass
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_mass * PARAMETER_SCALE)

        param9 = Parameter()
        param9.value = val
        param9.name = ACTIVE_CONTROLLER_MASS_PARAM

        #scale and int
        int_values = []
        for value in self.talos_rotational_inertias:
            int_values.append(int(value * PARAMETER_SCALE))

        #angular interias
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param10 = Parameter()
        param10.value = val
        param10.name = ACTIVE_CONTROLLER_ROTATIONAL_INERTIAS_PARAM

        #scale and int
        int_values = []
        for value in self.talos_vmax_values:
            int_values.append(int(value * PARAMETER_SCALE))

        #angular interias
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param11 = Parameter()
        param11.value = val
        param11.name = ACTIVE_CONTROLLER_VMAX_PARAM

        #scale and int
        int_values = []
        for value in self.talos_amax_values:
            int_values.append(int(value * PARAMETER_SCALE))

        #angular interias
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param12 = Parameter()
        param12.value = val
        param12.name = ACTIVE_CONTROLLER_AMAX_PARAM

        #scale and int
        int_values = []
        for value in self.talos_jmax_values:
            int_values.append(int(value * PARAMETER_SCALE))

        #angular interias
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param13 = Parameter()
        param13.value = val
        param13.name = ACTIVE_CONTROLLER_JAMX_PARAM
        
        #radial function count
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_radial_function_count * PARAMETER_SCALE)

        param14 = Parameter()
        param14.value = val
        param14.name = ACTIVE_CONTROLLER_RADIAL_FUNC_COUNT_PARAM

        #mass
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(PARAMETER_SCALE)

        param15 = Parameter()
        param15.value = val
        param15.name = ACTIVE_CONTROLLER_SCALING_PARAM

        #creste request to call the set parameters service
        request = SetParameters.Request()
        request.parameters = [param1, param2, param3, param4, param5, param6, param7, param8, param9, param10, param11, param12, param13, param14, param15]
        return request
    
    
    def getPIDSetParamRequest(self):
        #SET P GAINS
        int_values = []
        for value in self.talos_p_gains:
            int_values.append(int(value * PARAMETER_SCALE))
        
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param1 = Parameter()
        param1.value = val
        param1.name = ACTIVE_CONTROLLER_PID_P_GAINS_PARAM
        
        #SET I GAINS
        int_values = []
        for value in self.talos_i_gains:
            int_values.append(int(value * PARAMETER_SCALE))
        
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param2 = Parameter()
        param2.value = val
        param2.name = ACTIVE_CONTROLLER_PID_I_GAINS_PARAM
        
        #SET D GAINS
        int_values = []
        for value in self.talos_d_gains:
            int_values.append(int(value * PARAMETER_SCALE))
        
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
        val.integer_array_value = int_values

        param3 = Parameter()
        param3.value = val
        param3.name = ACTIVE_CONTROLLER_PID_D_GAINS_PARAM
        
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(self.talos_pid_reset_threshold * PARAMETER_SCALE)

        param4 = Parameter()
        param4.value = val
        param4.name = ACTIVE_CONTROLLER_PID_RESET_THRESHOLD_PARAM
        
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_INTEGER
        val.integer_value = int(PARAMETER_SCALE)

        param5 = Parameter()
        param5.value = val
        param5.name = ACTIVE_CONTROLLER_PID_SCALING_FACTOR_PARAM
        
        request = SetParameters.Request()
        request.parameters = [param1, param2, param3, param4, param5]
        return request
    

    def setActiveParams(self):
        self.get_logger().info("Setting Active Controller Default Parameters")
        #load in paramters to the active controller

        request = None
        if(self.activeControllerName == "SMC"):
            request = self.getSMCSetParamRequest()
        elif(self.activeControllerName == "PID"):
            request = self.getPIDSetParamRequest()
        
        if(request is None):
            self.get_logger().error(f"Internal error! Cannot set parameters for control model {self.activeControllerName} because it does not have a paramter function.")
            return

        #call set parameters service
        self.future = self.setActiveControllerParamsClient.call_async(request)

        #cancel the time that loads in paramters - don't spam reload
        self.paramTimerActive.cancel()


    def repubActiveParamsCb(self, msg):
        if(self.activeActive):
            self.get_logger().info("Preparing to reload Active Controller Params! Rereading Yaml!")

            self.readInRobotYaml()
            self.genertateThrusterForceMatrix(self.thruster_info, self.com)

            #reload paramters using a time to prevent spam reloading ;)
            self.get_logger().info("Parameters will be reloaded in 5 seconds!")
            self.paramTimerActive = self.create_timer(5.0, self.setActiveParams)
        else:
            self.get_logger().error("Failed to reload Active Params, the Active Controller is not active!")


    def repubThrusterSolverParamsCb(self, msg):
        if(self.solverActive):
            self.get_logger().info("Preparing to reload Thruster Solver Params! Rereading Yaml!")

            self.readInRobotYaml()
            self.genertateThrusterForceMatrix(self.thruster_info, self.com)

            #reload paramters using a time to prevent spam reloading ;)
            self.get_logger().info("Parameters will be reloaded in 5 seconds!")
            self.paramTimerTS = self.create_timer(5.0, self.setSolverParams)
        else:
            self.get_logger().error("Failed to reload Thruster Solver Params, the Thruster solver is not active!")

    #remove once matlab publishes to correct message
    def forceRepublishCb(self, msg:Int32MultiArray):
        #the message to republish to
        pub_msg = Float32MultiArray()

        if(len(msg.data) == 8):
            for datum in msg.data:
                #scale by .000001
                pub_msg.data.append(datum * .000001)


            #publish the message
            self.pubForceMsg = pub_msg
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


            self.pubRPMMsg = pub_msg
        else:
            self.get_logger().warn("RPM form solver has wrong length of: " + str(len(msg.data)))

    def pubCB(self):
        #publish rpms
        if(self.enabled):
            if(self.pubForceMsg is not None):
                self.forceCommandPub.publish(self.pubForceMsg)

            if(self.pubRPMMsg is not None):
                self.rpmCommandPub.publish(self.pubRPMMsg)

def main(args=None):
    rclpy.init(args=args)

    co = controllerOverseer()

    rclpy.spin(co)

if __name__ == '__main__':
    main()


