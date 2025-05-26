#! /usr/bin/env python3

import os
import yaml
import yaml.parser
from importlib import import_module
import rclpy
import numpy as np
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.time import Time
from tf2_ros.transform_listener import TransformListener
from tf2_ros import Buffer
from ros2node.api import get_node_names
from ament_index_python import get_package_share_directory
from tf_transformations import euler_matrix
from riptide_msgs2.msg import DshotPartialTelemetry
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int16, Int32MultiArray, Bool, Empty
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistWithCovariance

from overseer_global_defs import \
    PARAMETER_SCALE, SPECIAL_PARAMETERS, RELOAD_TIME, ORIN_AUTOTUNE_DIR, ACTIVE_PARAMETERS_MASK


# wrapper class for rclpy service which monitors for timeouts and prevents concurrent operations
# only one call is allowed at a time as these clients will be used to interact with parameters
class MonitoredServiceClient():    
    def __init__(self, node: Node, srv_type: type, srv_name: str):
        self.node = node
        self.client = node.create_client(srv_type, srv_name)
        self.timer = node.create_timer(0.5, self._timer_callback)
        
        self.waiting_for_client = False
        self.active_client = None #if active, should be a tuple of (request, callback)
        self.waiting_requests = deque([])
    
    
    def schedule_call(self, request, callback):
        if len(self.waiting_requests) < 10:
            self.waiting_requests.appendleft((request, callback))
            return True
        
        self.node.get_logger().error(f"Reached limit of calls for service {self.client.srv_name}")
        return False        
    
    
    def _timer_callback(self):
        current_time = self.node.get_clock().now()
        if self.waiting_for_client:
            # check if timed out
            if (current_time - self.srv_start_time).to_msg().sec >= 3:
                self.node.get_logger().error(f"Call to service {self.client.srv_name} timed out.")
                self.client.remove_pending_request(self.active_future)
                self.waiting_requests.pop()
                self.waiting_for_client = False
                self.active_client = None
        else:
            if len(self.waiting_requests) > 0:
                # able to schedule the next thing
                self.active_client = self.waiting_requests.pop()                
                (request, _) = self.active_client
                self.node.get_logger().debug(f"Making call to service {self.client.srv_name}")
                self.client.wait_for_service()
                self.active_future = self.client.call_async(request)
                self.active_future.add_done_callback(self._srv_callback)
                self.srv_start_time = current_time
    
    
    def _srv_callback(self, future: rclpy.Future):
        #pop queue, call callback
        if self.active_client is not None:
            (request, callback) = self.active_client
            self.active_client = None
            callback(future, request)
            self.waiting_for_client = False
        else:
            self.node.get_logger().error("MonitoredServiceClient received a callback but doesn't know where it came from")


#manages an individual simulink model.
class SimulinkModelNode():
    def __init__(self, node: 'ControllerOverseer', node_name: str, config, plugin_names: list[str]):
        self.model_active = False
        self.full_node_name = ""
        self.list_param_client = None
        self.set_param_client = None
        self.overseer_node = node
        self.node_name = node_name
        self.config = config
        self.known_params = []
        self.reload_param_service = node.create_service(Trigger, f"controller_overseer/update_{self.node_name}_params".lower(), self.reload_parameters_callback)
        self.last_reload_time = node.get_clock().now()
        self.loaded_params = False # have parameters been loaded
        self.loaded_plugins = False # have plugins been loaded
        
        # initialize plugins, but dont load them yet
        self.plugins = {}
        for plugin_name in plugin_names:
            factory = getattr(import_module(plugin_name), "get_plugin")
            self.plugins[plugin_name] = factory()
    
    # checks if the model is active. Handles when model comes up or goes down
    # IMPORTANT: active_nodes must be a list of the FULL node names (e.g. /talos/...).
    def check_if_active(self, active_nodes: 'list[str]'):
        active_model_nodes = [node for node in active_nodes if self.node_name in node]
        num_active_model_nodes = len(active_model_nodes)

        if num_active_model_nodes > 1:
            # not useful, but interesting. Print it out
            self.overseer_node.get_logger().warning(f"Detected {num_active_model_nodes} active nodes with the name {self.node_name}!")
        
        if num_active_model_nodes == 1:
            # if we got here, then we have a node we can control
            self.full_node_name = active_model_nodes[0]
            if not self.model_active:
                #note the model is active
                self.model_active = True                
                self.overseer_node.get_logger().info(f"Found {self.node_name} as {self.full_node_name}!")
                
                #model is present. now init required plugins if not already loaded
                if not self.loaded_plugins:
                    for plugin in self.plugins:
                        self.plugins[plugin].init_plugin(self.overseer_node, self.config, self)

                #the client to set the params of the model
                self.list_param_client = MonitoredServiceClient(self.overseer_node, ListParameters, f"{self.full_node_name}/list_parameters")
                self.set_param_client = MonitoredServiceClient(self.overseer_node, SetParameters, f"{self.full_node_name}/set_parameters")
                
                #initiate parameter reload to initialize default params
                self.reload_parameters()
            elif len(self.list_param_client.waiting_requests) == 0 and len(self.set_param_client.waiting_requests) == 0:
                # node already up and we are not trying to set parameters on it. check its params to make sure they are all set
                self.list_and_set_model_parameters()
            
        elif self.model_active:
            # model was active before, but not now
            self.model_active = False
            self.overseer_node.get_logger().warn(f"Lost {self.node_name}!")

    # resolves the available parameters on the model and then sets them.
    def list_and_set_model_parameters(self):
        if self.model_active and self.list_param_client is not None:
            return self.list_param_client.schedule_call(ListParameters.Request(), self.set_model_parameters_from_list_callback)
        
        self.overseer_node.get_logger().warning(f"Cannot list parameters for {self.node_name} because the model is not active or the list parameters client is None")
        return False
    
    
    # sets model parameters from the provided list
    def set_model_parameters(self, parameters: 'list[str]'):
        if self.model_active or self.node_name:
            rq = SetParameters.Request()
            param_array = []
            
            for parameter_name in parameters:
                val = ParameterValue()
                param = Parameter()
                
                #get parameter type and value
                read_value, type = self.get_param_value(parameter_name)
                if not read_value is None:
                    if type == int:
                        #set integer values
                        val.integer_value = int(PARAMETER_SCALE * read_value)
                        val.type = ParameterType.PARAMETER_INTEGER                    
                    
                    elif type == float:
                        #set float values
                        val.integer_value = int(PARAMETER_SCALE * read_value)
                        val.type = ParameterType.PARAMETER_INTEGER
                        
                    elif type == list:
                        #set list array values
                        int_val_array = []
                        for item in read_value:
                            int_val_array.append(int(item * PARAMETER_SCALE))

                        val.integer_array_value = int_val_array
                        val.type = ParameterType.PARAMETER_INTEGER_ARRAY
                    elif type == bool:
                        #set boolean parameters
                        val.bool_value = read_value
                        val.type = ParameterType.PARAMETER_BOOL

                    else:
                        self.overseer_node.get_logger().warn(f"Parameter type {type} not handled yet")

                    param.value = val
                    param.name = parameter_name
                    param_array.append(param)

                    self.overseer_node.get_logger().info(f"Setting {parameter_name} to {read_value}")
                else:
                    self.overseer_node.get_logger().warning(f"Not setting param {parameter_name}, no param found in config file!")
                    self.known_params.append(parameter_name)

            rq.parameters = param_array
            self.set_param_client.schedule_call(rq, self.set_parameters_done_callback)
        else:
            self.overseer_node.get_logger().warning(f"Cannot list parameters for {self.node_name} because the model is not active or the set parameters client is None")
    
    
    # can be used as a list parameters service callback which immediately attempts to set the available parameters
    # IMPORTANT: This callback will ONLY set parameters that are not present in the known_params list. To set a specific
    # parameter regardless of its state, use set_model_parameters with the names of the parameters you wish to set
    def set_model_parameters_from_list_callback(self, future: rclpy.Future, request):
        param_names = future.result().result.names
        unknown_param_names = [name for name in param_names if name not in self.known_params]
        if len(unknown_param_names) > 0:
            self.set_model_parameters(unknown_param_names)
        
    
    # callback for when parameter set completes
    def set_parameters_done_callback(self, future: rclpy.Future, request):
        result = future.result()
        fail_indices = [idx for idx in range(0, len(result.results)) if not result.results[idx].successful]
        
        for idx in range(0, len(result.results)):
            if not idx in fail_indices: #successful result, add it to known params
                self.known_params.append(request.parameters[idx].name)
        
        if len(fail_indices) == 0:
            self.overseer_node.get_logger().info(f"Successfully set parameters for {self.node_name}")
        else:
            for fail in fail_indices:
                self.overseer_node.get_logger().error(f"Failed to set parameter {request.parameters[fail].name} for {self.node_name}: {result.results[fail].reason}")

        self.loaded_params = True

    
    #gets a parameter value and type from the config
    def get_param_value(self, param_name):
        #check to see if the param is special
        if param_name in SPECIAL_PARAMETERS:
            if param_name == "talos_wrenchmat":
                return self.plugins["overseer_thruster_solver_plugin"].get1dWrenchmat()
            else:
                self.overseer_node.get_logger().warn("Undefine special parameter!")

        #load the parameter value from the config file
        config_tree = self.overseer_node.configTree
        split_path = str.split(param_name, "__")
        try:
            for part_path in split_path:
                config_tree = config_tree[part_path]
        except:
            return None, None
        else:
            #executed when no exception
            return config_tree, type(config_tree)
    
    
    # this function re-reads the config and loads the paramters onto the node.
    # to load the parameters WITHOUT re-reading the config, use list_and_set_model_parameters
    def reload_parameters(self):
        self.known_params = []
        self.overseer_node.readConfig()

        return self.list_and_set_model_parameters()
    
    
    def reload_parameters_callback(self, request, response):
        current_time = self.overseer_node.get_clock().now()
        if not self.model_active:
            response.success = False
            response.message = f"{self.node_name} is not active!"
            return response
        
        if (current_time - self.last_reload_time).to_msg().sec < RELOAD_TIME:
            response.success = False
            response.message = f"{self.node_name} has been reloaded within the last {RELOAD_TIME} seconds."
            return response
        
        success = self.reload_parameters()
        response.success = success
        response.message = f"{self.node_name} parameter reload triggered." if success else "Failed for unknown reason. Check overseer logs."
        
        if success:
            # set most recent call time to now
            self.last_reload_time = current_time
            
        return response
        
            

#manages parameters for the controller / thruster solver
class ControllerOverseer(Node):
    # waiting_on_init = False

    # drag_comp_forward_data = None
    # drag_comp_reverse_data = None

    # refresh_drag_file = True

    def __init__(self):
        super().__init__("controller_overseer")
        
        # self.escPowerStopsLow = 0
        # self.escPowerStopsHigh = 0
        # self.configTree = {}
        # self.currentAutoTuneTwist = [0,0,0,0,0,0]
        # self.autoff_config_path = None

        #
        # Declare parameters 
        #
        
        #get robot name
        self.declare_parameter("robot", "")
        self.robotName = self.get_parameter("robot").value

        #get robot config file path
        self.declare_parameter("vehicle_config", "")
        self.config_path = self.get_parameter("vehicle_config").value

        # #get thruster solver node name
        # self.declare_parameter("thruster_solver_node_name", "")
        # self.thrusterSolverName = self.get_parameter("thruster_solver_node_name").value

        #declare write autotune param
        # self.declare_parameter("write_ff_autotune", True)
        # self.writeFFAutoTune = self.get_parameter("write_ff_autotune").value

        # self.declare_parameter(FF_PUBLISH_PARAM, False)
        
        # Other setup
        
        #set the member configuration path variable
        self.setConfigPath()
        
        # read config
        self.readConfig()
        
        # tracked simulink models
        self.model_nodes = {
            "complete_controller": SimulinkModelNode(self, "complete_controller", self.configTree,
                [
                    "overseer_6dof_teleop_plugin",
                    "overseer_autotune_plugin",
                    "overseer_ff_plugin",
                    "overseer_thruster_solver_plugin"
                ]),
            "liltank_controller": SimulinkModelNode(self, "liltank_controller", self.configTree,
                [ ])
        }

        #generate thruster force matrix
        # self.generateThrusterForceMatrix(self.thruster_info, self.com)

        #default thruster weights and working thrusters
        # self.activeThrusters = [True, True, True, True, True, True, True, True]
        # self.submergedThrusters = [True, True, True, True, True, True, True, True]
        # self.thrusterWeights = [1, 1, 1, 1, 1, 1, 1, 1]

        # the thruster mode
        # self.thrusterMode = 1

        #declare pubs and subs

        #thruster telemetry
        # self.create_subscription(DshotPartialTelemetry, "state/thrusters/telemetry", self.thrusterTelemetryCB, qos_profile_system_default)

        #publish to active controllers whether motion is enabled
        # self.motionEnabledPub = self.create_publisher(Bool, "controller/motion_enabled", qos_profile=qos_profile_system_default)

        #thruster solver parameters
        # self.setThrusterSolverParamsClient = self.create_client(SetParameters, f"{self.thrusterSolverName}/set_parameters")

        #thruster mode
        # self.create_subscription(Int16, "thrusterSolver/thrusterState", self.setThrusterModeCB, qos_profile_system_default)

        #odometry filtered
        # self.create_subscription(Odometry, "odometry/filtered", self.odometryCB, qos_profile_system_default)

        #sub to autotune
        # self.create_subscription(Twist, "ff_auto_tune", self.ffAutoTuneCB, qos_profile_system_default)

        # #sub to the drag compensator forward
        # self.create_subscription(TwistWithCovariance, "controller/drag_comp/forward", self.dragCompForwardCb, qos_profile_system_default)

        # #sub to the drag compensator reverse
        # self.create_subscription(TwistWithCovariance, "controller/drag_comp/reverse", self.dragCompRevereseCb, qos_profile_system_default)

        #pub for thruster weights
        # self.weightsPub = self.create_publisher(Int32MultiArray, THRUSTER_SOLVER_WEIGHT_MATRIX_TOPIC, qos_profile_system_default)

        #pub ff force
        # self.ffPub = self.create_publisher(Twist, FF_TOPIC_NAME, qos_profile_system_default)

        #pub signal to reinit autoff and dragcal
        # self.re_init_signal_pub = self.create_publisher(Empty, AUTOTUNE_REINIT_TOPIC_NAME, qos_profile_system_default)

        #create teleop serivce
        # self.setTeleop = self.create_service(SetBool, "setTeleop", self.setTeleop)

        #declare transform
        # self.tfBuffer = Buffer()
        # self.tfListener = TransformListener(self.tfBuffer, self)

        # #the tf namespace
        # # TODO: FIX
        # # self.tfNamespace = self.get_parameter("robot").value
        # self.tfNamespace = "talos"

        #start time
        # self.startTime = None

        #timer to update models and ff
        self.create_timer(1.0, self.doUpdate)

        #start the publish loop
        # self.create_timer(WEIGHTS_FORCE_UPDATE_PERIOD, self.adjustThrusterWeights)

        # self.enabled = True
        self.publishingFF = True

        # # a timer to ensure that the active controllers stop if telemetry stops publishing!
        # self.escPowerCheckTimer = self.create_timer(ESC_POWER_TIMEOUT, self.escPowerTimeout)


    # def dragCompForwardCb(self, msg):

    #     #save drag data
    #     self.drag_comp_forward_data = msg.covariance
    #     self.refresh_drag_file = True

    # def dragCompRevereseCb(self, msg):

    #     #save drag data
    #     self.drag_comp_reverse_data = msg.covariance
    #     self.refresh_drag_file = True



    # def generateThrusterForceMatrix(self, thruster_info, com):
    #     #generate the thruster effect matrix
    #     self.thrusterEffects = np.zeros(shape=(8,6))
    #     for i, thruster in enumerate(thruster_info):
    #         pose = np.array(thruster["pose"])

    #         #calculate the force vector generated by the thruster
    #         forceVector = np.matmul(euler_matrix(pose[3], pose[4], pose[5])[:3, :3], np.array([1,0,0]))
    #         positionFromCOM = pose[:3] - com
    #         torque = np.cross(positionFromCOM[:3], forceVector)

    #         #insert into thruster effect matrix
    #         self.thrusterEffects[i] = [forceVector[0], forceVector[1], forceVector[2], torque[0], torque[1], torque[2]]

    def setConfigPath(self):
        #set the path to the config file
        #reload in file from scratch just incase the target yaml has changed
        self.configPath = self.get_parameter("vehicle_config").value
        if self.configPath == '':
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

        #set the path to the autocal saved data file
        control_share_dir = get_package_share_directory("riptide_controllers2")

        auto_ff_subpath = os.path.join("config", self.robotName + "_autoff.yaml")

        if not os.path.exists("/home/ros/colcon_deploy"):
            #running on personal computer
            self.get_logger().info("I think I am running on a development laptop!")

            self.autoff_config_path = os.path.join(control_share_dir, auto_ff_subpath)

        else:
            self.get_logger().info("I think I am running on the orin!")

            self.autoff_config_path = os.path.join(ORIN_AUTOTUNE_DIR, self.robotName + "_autoff.yaml")

    def readConfig(self):
        try:
            with open(self.configPath, "r") as config:
                self.configTree = yaml.safe_load(config)
        except:
            self.get_logger().error(f"Cannot open config file at {self.configPath}!")

        try:
            with open(self.autoff_config_path, "r") as config:
                autoff_yaml_data = yaml.safe_load(config)
                autoff_init = autoff_yaml_data["auto_ff"]
              
                #ensure the re init signal is sent to the controller mode;  
                self.waiting_on_init = True

                auto_loaded_params = dict()
                auto_loaded_params["initial_ff"] = autoff_init
                self.configTree["controller"]["autoff"] = auto_loaded_params

                drag_forward_init = autoff_yaml_data["drag_forward"]
                drag_reverse_init = autoff_yaml_data["drag_reverse"]

                drag_initial_comp = dict()
                drag_initial_comp["forward"] = drag_forward_init
                drag_initial_comp["reverse"] = drag_reverse_init
                
                self.configTree["controller"]["drag_compensator"]["initial_compensation"] = drag_initial_comp
                
                self.get_logger().info(f"{self.configTree}")
                
                self.currentAutoTuneTwist = autoff_init
        except FileNotFoundError:
            self.get_logger().error(f"Cannot open config file at {self.autoff_config_path}!")
        except KeyError:
            self.get_logger().error(f"Missing Autoff Keys!")
        except TypeError:
            self.get_logger().error(f"Type Error!")
        except yaml.parser.ParserError:
            self.get_logger().error(f"Yaml Parser")
        
        # read specific values used by the class
                
        # thruster info
        # self.thruster_info = self.configTree['thrusters']
        
        #read in kill plane height
        # self.killPlane = self.configTree["controller_overseer"]["thruster_kill_plane"]
        
        # read in com data
        # self.com = self.configTree["com"]
        
        # feed-forward
        # self.base_wrench = self.configTree["controller"]["feed_forward"]["base_wrench"]
        
        #thruster solver
        # thruster_solver_info = self.configTree["thruster_solver"]
        
        # #read in weight info
        # self.defaultWeight      = thruster_solver_info["default_weight"]
        # self.surfaceWeight      = thruster_solver_info["surfaced_weight"]
        # self.disabledWeight     = thruster_solver_info["disable_weight"]
        # self.lowDowndraftWeight = thruster_solver_info["low_downdraft_weight"]


    # def thrusterTelemetryCB(self, msg: DshotPartialTelemetry):
    #     #wether or not the thrusterSolverweigths need adjusted
    #     adjustWeights = False

    #     #restart the timeout
    #     self.escPowerCheckTimer.reset()

    #     # which thrusters - groups of 4
    #     if msg.start_thruster_num == 0:
    #         #check wether each thruster is active
    #         for i, esc in enumerate(msg.esc_telemetry):
    #             if not esc.thruster_ready:
    #                 if self.activeThrusters[i] == True:
    #                     #if change, adjust the thruster weights
    #                     self.activeThrusters[i] = False
    #                     adjustWeights = True
    #             else:
    #                 if self.activeThrusters[i] == False:
    #                     self.activeThrusters[i] = True
    #                     adjustWeights = True

    #         #check if the boards are enebaled
    #         if not (msg.disabled_flags == 0):  
    #             self.escPowerStopsLow += 1    
    #         else:
    #             self.escPowerStopsLow = 0 

    #     else:
    #         for i, esc in enumerate(msg.esc_telemetry):
    #             if not esc.thruster_ready:
    #                 if self.activeThrusters[i + 4] == True:
    #                     self.activeThrusters[i + 4] = False
    #                     adjustWeights = True
    #             else:
    #                 if self.activeThrusters[i + 4] == False:
    #                     self.activeThrusters[i + 4] = True
    #                     adjustWeights = True

    #         #check if the boards are enebaled
    #         if not (msg.disabled_flags == 0):  
    #             self.escPowerStopsHigh += 1  
    #         else:
    #             self.escPowerStopsHigh = 0     

    #     #adjust the weights of the thruster solver if anything has changed
    #     if adjustWeights:
    #         self.adjustThrusterWeights()

    #     motionMsg = Bool()
    #     #check if the boards are enebaled
    #     if self.escPowerStopsLow > ESC_POWER_STOP_TOLERANCE or self.escPowerStopsHigh > ESC_POWER_STOP_TOLERANCE:
    #         #publish disabled message
    #         motionMsg.data = False
    #         self.enabled = False

    #         # if self.enabled:
    #         #     self.get_logger().warn("Recieving disabled flags from ESC!")
    #     else:

    #         motionMsg.data = True
    #         self.enabled = True
    #         #publish enabled message
        
    #     self.motionEnabledPub.publish(motionMsg)


    # def escPowerTimeout(self):
    #     #timeout for if the escs go to long without publishing telemerty
    #     if self.enabled:
    #         self.get_logger().warn("Not recieving thruster telemetry!")

    #     motionMsg = Bool()
    #     motionMsg.data = False
    #     self.enabled = False
    #     self.motionEnabledPub.publish(motionMsg)


    # def setThrusterModeCB(self, msg:Int16):
    #     #change the thruster mode

    #     #Modes -----
    #     #   1 - Normal
    #     #   2 - Low Downdraft

    #     if not (msg.data == self.thrusterMode):
    #         self.thrusterMode = msg.data

    #         #update the weights
    #         self.adjustThrusterWeights()


    # def odometryCB(self, msg):
    #     #check if thrusters are submerged - everytime odom is updated - just using as a frequency

    #     #start the start time on first cb
    #     if self.startTime is None:
    #         self.startTime = self.get_clock().now()

    #     submerged = [False, False, False, False, False, False, False, False]

    #     try:
    #         #look at each thrusterbreak
    #         for i, thursterSufaced in enumerate(self.submergedThrusters):
    #             pos = self.tfBuffer.lookup_transform("world", f"{self.tfNamespace}/thruster_{i}", Time())

    #             #if thruster is above the kill plane
    #             if pos.transform.translation.z < self.killPlane:
    #                 submerged[i] = True

    #         if not (np.array_equal(submerged, self.submergedThrusters)):
    #             #if a different thruster combo is submerdged, adjust, the weights
    #             self.submergedThrusters = submerged
    #             self.adjustThrusterWeights()

    #     except Exception as ex:
    #         if self.get_clock().now().to_msg().sec >= ERROR_PATIENCE + self.startTime.to_msg().sec:
    #             self.get_logger().error("Thruster Position Lookup failed with exception: " + str(ex))


    # def adjustThrusterWeights(self):
    #     #TODO add weight values into descriptions

    #     #number of active thrusters
    #     activeThrusterCount = 0
    #     submergedThrusters = 0

    #     #shutoff inactive thrusters - disabled or broken
    #     for i, isActive in enumerate(self.activeThrusters):
    #         if isActive:
    #             activeThrusterCount += 1

    #             #play around with weights for thrusters above surface
    #             if not (self.submergedThrusters[i] == True):
    #                 #if thruster is not submerdged
    #                 self.thrusterWeights[i] = self.surfaceWeight
    #             else:
    #                 #thruster is active and submerdged
    #                 submergedThrusters += 1
    #                 self.thrusterWeights[i] = self.defaultWeight

    #         else:
    #             #if a thruster is inactive - raise the cost of "using" thruster
    #             self.thrusterWeights[i] = self.disabledWeight

    #     if activeThrusterCount <= 6:
    #         #if system is not full actuated, it cannot be optimized, very high rpms / force can be requested
    #         #for the safety of the system, this will autodisable robot (probably)
    #         #remove if PIA
    #         if self.enabled:
    #             self.get_logger().error("System has become underactuated. Only:  " + str(activeThrusterCount) + " thrusters are active. Killing Thrusters!")
    #             self.enabled = False
    #     else:
    #         self.enabled = True

    #     if submergedThrusters >= 8:
    #         #take into account control modes only if all actuators are working

    #         if self.thrusterMode == 2:
    #             #apply low downdraft
    #             self.thrusterWeights[4] = self.lowDowndraftWeight
    #             self.thrusterWeights[5] = self.lowDowndraftWeight

    #     #impose disable weight if nessecary
    #     if self.thrusterMode == 0:
    #         self.thrusterWeights = [0,0,0,0,0,0,0,0]

    #     msg = Int32MultiArray()

    #     #scale and round all weights
    #     weights = []
    #     for weight in self.thrusterWeights:
    #         weights.append(int(weight))

    #     msg.data = weights

    #     #publish weights
    #     self.weightsPub.publish(msg)

    # def setTeleop(self, request, future):
    #     #set the teleop mode

    #     #if setting Teleop on
    #     if(request.data == True):
    #         try:
    #             #set the control mask values
    #             pVal = ParameterValue()
    #             pVal.type = ParameterType.PARAMETER_INTEGER_ARRAY
    #             pVal.integer_array_value = [3000000, 3000000, 2000000, 1000000, 1000000, 3000000]

    #             param = Parameter()
    #             param.value = pVal
    #             param.name = ACTIVE_PARAMETERS_MASK

    #             request = SetParameters.Request()
    #             request.parameters = [param]

    #             self.model_nodes["complete_controller"].set_param_client.schedule_call(request, self.model_nodes["complete_controller"].set_parameters_done_callback)

    #             future.success = True
    #             future.message = "Successfully enabled teleop!"

    #         except Exception as E:

    #             if(type(E) == AttributeError):
    #                 #this is an expected error so elaborate
    #                 self.get_logger().warn("Failed to initialize teleop, model is not started!")
    #                 future.success = False
    #                 future.message = "Failed to initialize teleop, model is not started!"
    #             else:
    #                 self.get_logger().warn(f"Failed to initialize teleop! {E}")
    #                 future.success = False
    #                 future.message = f"Failed to initialize teleop! {E}" 
    #     else:
    #     #putting into active control
    #         try:
    #             #set the control mask values
    #             pVal = ParameterValue()
    #             pVal.type = ParameterType.PARAMETER_INTEGER_ARRAY
    #             pVal.integer_array_value = [2000000, 2000000, 2000000, 1000000, 1000000, 2000000]

    #             param = Parameter()
    #             param.value = pVal
    #             param.name = ACTIVE_PARAMETERS_MASK

    #             request = SetParameters.Request()
    #             request.parameters = [param]

    #             self.model_nodes["complete_controller"].set_param_client.schedule_call(request, self.model_nodes["complete_controller"].set_parameters_done_callback)

    #             future.success = True
    #             future.message = "Successfully enabled active control!"

    #         except Exception as E:

    #             if(type(E) == AttributeError):
    #                 #this is an expected error so elaborate
    #                 self.get_logger().warn("Failed to initialize active control, model is not started!")
    #                 future.success = False
    #                 future.message = "Failed to initialize active control, model is not started!"
    #             else:
    #                 self.get_logger().warn(f"Failed to initialize active control! {E}")
    #                 future.success = False
    #                 future.message = f"Failed to initialize active control! {E}" 
                
    #     return future
            

    def doUpdate(self):
        # check model statuses
        active_rosnodes = get_node_names(node=self)
        active_rosnode_names = [node.namespace + "/" + node.name for node in active_rosnodes]   
        
        #remove double leading / which would happen in nodes in the root namespace
        active_rosnode_names = [node_name[1:] if node_name.startswith("//") else node_name for node_name in active_rosnode_names]
             
        for model_node in self.model_nodes.values():
            model_node.check_if_active(active_rosnode_names)        
        
        
        # # publish ff if necessary
        # if not (self.get_parameter(FF_PUBLISH_PARAM).value):
        #     #publish the ff
        #     self.publishingFF = True

        #     msg = Twist()
        #     msg.linear.x = self.base_wrench[0]
        #     msg.linear.y = self.base_wrench[1]
        #     msg.linear.z = self.base_wrench[2]
        #     msg.angular.x = self.base_wrench[3]
        #     msg.angular.y = self.base_wrench[4]
        #     msg.angular.z = self.base_wrench[5]

        #     self.ffPub.publish(msg)
        # elif self.publishingFF == True:
        #     #send zero before finishing
        #     self.publishingFF = False
        #     msg = Twist()
        #     msg.linear.x = 0.0
        #     msg.linear.y = 0.0
        #     msg.linear.z = 0.0
        #     msg.angular.x = 0.0
        #     msg.angular.y = 0.0
        #     msg.angular.z = 0.0

        #     self.ffPub.publish(msg)

    # def ffAutoTuneCB(self, msg):
    #     #write the autotune save if it has changed

    #     #if writing is disabled
    #     if not self.writeFFAutoTune:
    #         return
        
    #     #if the overseer has not initialized the controller yet
    #     if not self.model_nodes["complete_controller"].loaded_params:
    #         return
    
    #     #if the auto ff config path doesn't exist
    #     if (self.autoff_config_path == None):
    #         return 

    #     #if the controller is trying to reinit the autoff
    #     if(self.waiting_on_init):

    #         auto_ff_init = None
    #         try:
    #             auto_ff_init = self.configTree["controller"]["autoff"]["initial_ff"]
    #         except KeyError:
    #             self.get_logger().warn("Cannot find the initial auto ff in config tree!")
    #             return

    #         if (abs(auto_ff_init[0] - msg.linear.x) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[1] - msg.linear.y) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[2] - msg.linear.z) < AUTOFF_INIT_TOLERANCE and 
    #             abs(auto_ff_init[3] - msg.angular.x) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[4] - msg.angular.y) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[5] - msg.angular.z) < AUTOFF_INIT_TOLERANCE):

    #             #if the init signal has taken
    #             self.waiting_on_init = False

    #         else: 
    #             #pub msg
    #             msg = Empty()
    #             self.re_init_signal_pub.publish(msg)

    #         return

    #     #if the twist has been updated
    #     if (not (self.currentAutoTuneTwist[0] == msg.linear.x and self.currentAutoTuneTwist[1] == msg.linear.y and self.currentAutoTuneTwist[2] == msg.linear.z and 
    #        self.currentAutoTuneTwist[3] == msg.angular.x and self.currentAutoTuneTwist[4] == msg.angular.y and self.currentAutoTuneTwist[5] == msg.angular.z)) or (self.refresh_drag_file):
    #         #prepare string to be wrote
    #         auto_ff_config_string = f"auto_ff: [{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}]\n"
            
    #         drag_forward_string = ""
    #         drag_reverse_string = ""
    #         if(not ((self.drag_comp_forward_data is None) or (self.drag_comp_reverse_data is None))):

    #             #write the forward drag string
    #             drag_forward_string = f"drag_forward: ["
    #             for value in self.drag_comp_forward_data:
    #                 drag_forward_string = drag_forward_string + str(value) + ","
    #             drag_forward_string =  drag_forward_string + "]\n"

    #             #write the reverse drag string
    #             drag_reverse_string = f"drag_reverse: ["
    #             for value in self.drag_comp_reverse_data:
    #                 drag_reverse_string = drag_reverse_string + str(value) + ","
    #             drag_reverse_string =  drag_reverse_string + "]\n"

    #         #write config to file
    #         try:
    #             with open(self.autoff_config_path, "w") as config:
                    
    #                 config.write(auto_ff_config_string)
                    

    #                 if not ((self.drag_comp_forward_data is None) or (self.drag_comp_reverse_data is None)):
    #                  #cant save until cb complete
    #                     config.write(drag_forward_string)
    #                     config.write(drag_reverse_string)
                        
    #                 config.close()

    #         except FileExistsError:
    #             self.get_logger().error(f"Cannot open ff auto tune file at: {self.autoff_config_path}")
    #         except PermissionError:
    #             self.get_logger().error(f"No Permission to write of autoff file at: {self.autoff_config_path}")

        
                
def main(args=None):
    rclpy.init(args=args)

    co = ControllerOverseer()

    rclpy.spin(co)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
