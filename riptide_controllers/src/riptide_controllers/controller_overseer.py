#! /usr/bin/env python3

import os
from subprocess import run, PIPE, TimeoutExpired

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
from rcl_interfaces.srv import SetParameters, ListParameters, DescribeParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int16, Int32MultiArray, Float32MultiArray, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from rclpy.time import Time

ERROR_PATIENCE = 1.0

#the number of frames the escs can be powered off before clearing acculators
ESC_POWER_STOP_TOLERANCE = 2
ESC_POWER_TIMEOUT = 2

PARAMETER_SCALE = 1000000

ACTIVE_CONTROLLER_NAMES = ["PID", "SMC"]

THRUSTER_SOLVER_WEIGHT_MATRIX_TOPIC = "controller/solver_weights"

#paramters that cannot be pulled from yaml but still need setting
SPECIAL_PARAMTERS = ["talos_wrenchmat"]

FF_PUBLISH_PARAM = "disable_native_ff"
FF_TOPIC_NAME = "controller/FF_body_force"
ACTIVE_CONTROL_FORCE_TOPIC_NAME = "/talos/controller/active_body_force"

RPM_PUBLISH_PERIOD = .02
WEIGHTS_FORCE_UPDATE_PERIOD = 1

G = 9.8067

#manages parameters for the controller / thruster solver
class controllerOverseer(Node):

    escPowerStopsLow = 0
    escPowerStopsHigh = 0

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

        #get pid node name
        self.declare_parameter("pid_model_name", "PID")
        self.pidName = self.get_parameter("pid_model_name").value
        
        #get smc node name
        self.declare_parameter("smc_model_name", "SMC")
        self.smcName = self.get_parameter("smc_model_name").value

        self.declare_parameter("complete_model_name", "complete_controller")
        self.complete_name = self.get_parameter("complete_model_name").value

        self.declare_parameter(FF_PUBLISH_PARAM, False)

        #set the member configuration path variable
        self.setConfigPath()

        #read in yaml
        self.readInRobotYaml()

        #generate thruster force matrix
        self.generateThrusterForceMatrix(self.thruster_info, self.com)

        #default thruster weights and working thrusters
        self.activeThrusters = [True, True, True, True, True, True, True, True]
        self.submerdgedThrusters = [True, True, True, True, True, True, True, True]
        self.thrusterWeights = [1,1,1,1,1,1,1,1]

        # the thruster mode
        self.thrusterMode = 1

        self.param_set_clear = True

        #declare pubs and subs

        #thruster telemetry
        self.create_subscription(DshotPartialTelemetry, "state/thrusters/telemetry", self.thrusterTelemetryCB, qos_profile_system_default)

        #publish to active controllers whether motion is enabled
        self.motionEnabledPub = self.create_publisher(Bool, "controller/motion_enabled", qos_profile=qos_profile_system_default)

        #thruster solver parameters
        self.setThrusterSolverParamsClient = self.create_client(SetParameters, f"{self.thrusterSolverName}/set_parameters")

        #thruster mode
        self.create_subscription(Int16, "thrusterSolver/thrusterState", self.setThrusterModeCB, qos_profile_system_default)

        #odometry filtered
        self.create_subscription(Odometry, "odometry/filtered", self.odometryCB, qos_profile_system_default)

        #pub for thruster weights
        self.weightsPub = self.create_publisher(Int32MultiArray, THRUSTER_SOLVER_WEIGHT_MATRIX_TOPIC, qos_profile_system_default)

        #pub ff force
        self.ffPub = self.create_publisher(Twist, FF_TOPIC_NAME, qos_profile_system_default)
        
        #services to republish params
        
        self.repubSrvActive = self.create_service(Trigger, "controller_overseer/update_smc_params", lambda: self.reloadParams("/talos/SMC"))
        self.repubSrvActive = self.create_service(Trigger, "controller_overseer/update_pid_params", lambda: self.reloadParams("/talos/PID"))
        self.repubSrvThruster = self.create_service(Trigger, "controller_overseer/update_ts_params", lambda: self.reloadParams("/talos/thruster_solver"))

        #timestamps to prevent spamming of param repub
        self.lastRepubPIDTime = self.get_clock().now()
        self.lastRepubSMCTime = self.get_clock().now()
        self.lastRepubThrusterTime = self.get_clock().now()

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
        self.pidActive = False
        self.smcActive = False
        
        #is the complete controller active
        self.completeActive = False

        #attempt to set the wrench matrix in the thruster solver
        self.create_timer(1.0, self.checkIfSimulinkActive)

        #publsh msgs at frequency
        self.pubRPMMsg = None
        self.pubForceMsg = None

        #start the publish loop
        self.create_timer(WEIGHTS_FORCE_UPDATE_PERIOD, self.adjustThrusterWeights)

        self.pubTimer = None
        self.enabled = True
        self.publishingFF = True


        #a timer to ensure that the active controllers stop if telemtry stops publishing!
        self.escPowerCheckTimer = self.create_timer(ESC_POWER_TIMEOUT, self.escPowerTimeout)

    def generateThrusterForceMatrix(self, thruster_info, com):
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

    def setConfigPath(self):
        #set the path to the config file
        #reload in file from scratch just incase the target yaml has changed
        self.configPath = self.get_parameter("vehicle_config").value
        if(self.configPath == ''):
            # Try to locate the source directory, and use that configuration file directly
            descriptions_share_dir = get_package_share_directory("riptide_descriptions2")

            robot_config_subpath = os.path.join("config", self.robotName + ".yaml")

            # Set fallback config path if we can't find the one in source
            self.configPath = os.path.join(descriptions_share_dir, robot_config_subpath)

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
                        self.configPath = path_check
                        self.get_logger().info(f"Discovered source directory, overriding descriptions to use '{self.configPath}'")
                        break

    def readInRobotYaml(self):

        with open(self.configPath, "r") as config:
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
        
        #read in solver mask param
        self.activeTwistMask = thruster_solver_info["active_force_mask"]

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
        
        #gravity well params (replacing motion profiles)
        self.talos_velocity_curve_params = config_file["controller"]["SMC"]["SMC_params"]["velocity_curve_params"]
        
        #motion generation params (unused. TODO: DELETE)
        self.talos_vmax_values = config_file["controller"]["SMC"]["motion_profile_params"]["max_velocity"]
        self.talos_amax_values = config_file["controller"]["SMC"]["motion_profile_params"]["max_acceleration"]
        self.talos_jmax_values = config_file["controller"]["SMC"]["motion_profile_params"]["max_jerk"]
        self.talos_radial_function_count = config_file["controller"]["SMC"]["motion_profile_params"]["radial_function_count"]
        
        #PID specific parameters
        self.talos_p_gains = config_file["controller"]["PID"]["p_gains"]
        self.talos_i_gains = config_file["controller"]["PID"]["i_gains"]
        self.talos_d_gains = config_file["controller"]["PID"]["d_gains"]        
        self.talos_p_surface_gains = config_file["controller"]["PID"]["p_surface_gains"]
        self.talos_i_surface_gains = config_file["controller"]["PID"]["i_surface_gains"]
        self.talos_d_surface_gains = config_file["controller"]["PID"]["d_surface_gains"]
        self.talos_max_control = config_file["controller"]["PID"]["max_control_threshholds"]
        self.talos_surface_gain_floor = config_file["controller"]["PID"]["surface_gain_floor"]
        self.talos_surface_gain_buffer = config_file["controller"]["PID"]["surface_gain_buffer"]
        self.talos_pid_reset_threshold = config_file["controller"]["PID"]["reset_threshold"]


    def thrusterTelemetryCB(self, msg: DshotPartialTelemetry):
        #wether or not the thrusterSolverweigths need adjusted
        adjustWeights = False

        #restart the timeout
        self.escPowerCheckTimer.reset()

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

            #check if the boards are enebaled
            if not (msg.disabled_flags == 0):  
                self.escPowerStopsLow += 1    
            else:
                self.escPowerStopsLow = 0 

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

            #check if the boards are enebaled
            if not (msg.disabled_flags == 0):  
                self.escPowerStopsHigh += 1  
            else:
                self.escPowerStopsHigh = 0     

        #adjust the weights of the thruster solver if anything has changed
        if(adjustWeights):
            self.adjustThrusterWeights()

        motionMsg = Bool()
        #check if the boards are enebaled
        if self.escPowerStopsLow > ESC_POWER_STOP_TOLERANCE or self.escPowerStopsHigh > ESC_POWER_STOP_TOLERANCE:
            #publish disabled message
            motionMsg.data = False

            self.get_logger().warn("Recieving disabled flags from ESC!")
        else:

            motionMsg.data = True
            #publish enabled message
        
        self.motionEnabledPub.publish(motionMsg)

    def escPowerTimeout(self):
        #timeout for if the escs go to long without publishing telemerty
        self.get_logger().warn("Not recieving thruster telemtry!")

        motionMsg = Bool()
        motionMsg.data = False
        self.motionEnabledPub.publish(motionMsg)


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
            #look at each thrusterbreak
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

        #see if the matlab node is running
        found_thruster_solver = False
        found_PID = False
        found_SMC = False
        found_complete = False

        for nodeName in get_node_names(node=self):

            #check the thruster solver
            if self.thrusterSolverName in nodeName.name:

                #mark as found!
                found_thruster_solver = True

                #if the solver previously wasn't active
                if(not self.solverActive and self.param_set_clear):

                    #note the solver is active
                    self.get_logger().info("Found Sovler")

                    self.solverActive = True

                    #set so no other model can being having its params set
                    self.param_set_clear = False

                    #the name of the node currently having its params set
                    self.working_model_node_name = "/" + nodeName.name
                    if(nodeName.namespace is not None):
                        if(nodeName.namespace != '' or nodeName.namespace != '/'):
                            self.working_model_node_name = nodeName.namespace + "/" + nodeName.name

                    self.get_logger().info("self.working_model_node_name")

                    #the client to set the params of the model
                    self.set_param_client = self.create_client(SetParameters, f"{self.working_model_node_name}/set_parameters")

                    #set the model's parameters
                    self.set_model_paramters_timer = self.create_timer(0.5, self.setModelParameters)


            #check the active controller
            if self.pidName in nodeName.name:

                #mark as found!
                found_PID= True

                #if the solver previously wasn't active
                if(not self.pidActive and self.param_set_clear):
                    #note the pid is active
                    self.get_logger().info("Found PID")

                    self.pidActive = True

                    #set so no other model can being having its params set
                    self.param_set_clear = False

                    #the name of the node currently having its params set
                    self.working_model_node_name = "/" + nodeName.name
                    if(nodeName.namespace is not None):
                        if(nodeName.namespace != '' or nodeName.namespace != '/'):
                            self.working_model_node_name = nodeName.namespace + "/" + nodeName.name

                    #the client to set the params of the model
                    self.set_param_client = self.create_client(SetParameters, f"{self.working_model_node_name}/set_parameters")

                    #set the model's parameters
                    self.set_model_paramters_timer = self.create_timer(0.5, self.setModelParameters)


                
            if self.smcName in nodeName.name:

                #mark as found!
                found_SMC = True

                #if the solver previously wasn't active
                if(not self.solverActive and self.param_set_clear):
                    #note the smc is active
                    self.get_logger().info("Found SMC")

                    self.smcActive = True

                    #set so no other model can being having its params set
                    self.param_set_clear = False

                    #the name of the node currently having its params set
                    self.working_model_node_name = "/" + nodeName.name
                    if(nodeName.namespace is not None):
                        if(nodeName.namespace != '' or nodeName.namespace != '/'):
                            self.working_model_node_name = nodeName.namespace + "/" + nodeName.name

                    #the client to set the params of the model
                    self.set_param_client = self.create_client(SetParameters, f"{self.working_model_node_name}/set_parameters")

                    #set the model's parameters
                    self.set_model_paramters_timer = self.create_timer(0.5, self.setModelParameters)



            if self.complete_name in nodeName.name:

                #mark as found!
                found_complete = True

                #if the solver previously wasn't active
                if(not self.completeActive and self.param_set_clear):
                    #note the solver is active
                    self.get_logger().info("Found Complete Controller")

                    self.completeActive = True

                    #set so no other model can being having its params set
                    self.param_set_clear = False

                    #the name of the node currently having its params set
                    self.working_model_node_name = "/" + nodeName.name
                    if(nodeName.namespace is not None):
                        if(nodeName.namespace != '' and nodeName.namespace != '/'):
                            self.working_model_node_name = nodeName.namespace + "/" + nodeName.name

                    #the client to set the params of the model
                    self.set_param_client = self.create_client(SetParameters, f"{self.working_model_node_name}/set_parameters")

                    #set the model's parameters
                    self.set_model_paramters_timer = self.create_timer(0.5, self.setModelParameters)

        if not found_thruster_solver and self.solverActive:
            #complete is disactive
            self.solverActive = False
            self.get_logger().warn("Lost Thruster Solver!")

        if not found_SMC and self.smcActive:
            #complete is disactive
            self.smcActive = False
            self.get_logger().warn("Lost SMC!")

        if not found_PID and self.pidActive:
            #complete is disactive
            self.pidActive = False
            self.get_logger().warn("Lost PID!")

        if not found_complete and self.completeActive:
            #complete is disactive
            self.completeActive = False
            self.get_logger().warn("Lost Complete Controller!")

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

    def reloadParams(self, param_name):
        #reload params for a model

        while(not self.param_set_clear):
            #ensure no one else is currently setting parameters
            self.param_set_clear = True
            self.working_model_node_name = param_name

            self.set_model_paramters_timer = self.create_timer(0.5, self.setModelParameters)

    def setModelParameters(self):
        #cancel timer that called
        self.set_model_paramters_timer.cancel()

        result = None

        try:
            cmd = [f"ros2 service call {self.working_model_node_name}/list_parameters rcl_interfaces/srv/ListParameters"]
            result = run(cmd, stdout=PIPE, stderr=PIPE, timeout=3, shell=True)
            #stupid string logic
        except TimeoutExpired as TE:
            self.get_logger().info("Timed out service, reattempting")
            self.set_model_paramters_timer = self.create_timer(3, self.setModelParameters)
            return
                
        try:
            param_names = str.split(str.split(str.split(str(result), "names=['")[1], "']")[0], "', '")

            self.param_set_timer = self.create_timer(0.1, lambda: self.setParamsFromNames(param_names))
        except IndexError as e:
            self.get_logger().error("Failed to parse parms:" + str(result))
            self.param_set_clear = True


    def setParamsFromNames(self, names):
        #set the paramters using the name path method

        rq = SetParameters.Request()
        param_array = []

        for parameter_name in names:
            val = ParameterValue()
            param = Parameter()
            
            #get parameter type adn value
            read_value, type = self.getParamValue(parameter_name)
            if(not read_value is None):
                if type == '''<class 'int'>''':
                    #set integer values
                    val.integer_value = int(PARAMETER_SCALE * read_value)
                    val.type = ParameterType.PARAMETER_INTEGER                    
                
                elif type == '''<class 'float'>''':
                    #set integer values
                    val.integer_value = int(PARAMETER_SCALE * read_value)
                    val.type = ParameterType.PARAMETER_INTEGER
                    
                elif type == '''<class 'list'>''':
                    #set integer array values

                    int_val_array = []
                    for item in read_value:
                        int_val_array.append(int(item * PARAMETER_SCALE))

                    val.integer_array_value = int_val_array
                    val.type = ParameterType.PARAMETER_INTEGER_ARRAY
                elif type == '''<class 'bool'>''':
                    #set boolean parameters

                    val.bool_value = read_value
                    val.type = ParameterType.PARAMETER_BOOL

                else:
                    self.get_logger().warn(f"Paramter type {type} not handled yet")

                param.value = val
                param.name = parameter_name
                param_array.append(param)

                self.get_logger().info(f"Setting {parameter_name} to {val}")

            else:
                self.get_logger().info(f"Not setting param {parameter_name}, no param found in config file!")

        rq.parameters = param_array
        self.future = self.set_param_client.call_async(rq)

        self.param_set_timer.cancel()
        self.param_set_clear = True
    
    def getParamValue(self, param_name):
        #check to see if the param is special
        if param_name in SPECIAL_PARAMTERS:
            if param_name == "talos_wrenchmat":
                #change to 1D array
                effects_1D = []
                for column in self.thrusterEffects:
                    for effect in column:
                        effects_1D.append(effect)

                return effects_1D, '''<class 'list'>'''
            else:
                self.get_logger().warn("Undefine special parameter!")

        #load the paramter value from the config file
        split_path = str.split(param_name, "__")

        try:
            with open(self.configPath, "r") as config:
                config_tree = yaml.safe_load(config)

                try:
                    for part_path in split_path:
                        config_tree = config_tree[part_path]
                except:
                    return None, None
                else:
                    return config_tree, str(type(config_tree))

        except:
            self.get_logger().warn("Cannot open config file!")
            return None, None
                
                
def main(args=None):
    rclpy.init(args=args)

    co = controllerOverseer()

    rclpy.spin(co)

if __name__ == '__main__':
    main()


