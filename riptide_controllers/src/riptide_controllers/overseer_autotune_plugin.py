from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TwistWithCovariance

from overseer_global_defs import AUTOTUNE_REINIT_TOPIC_NAME, FF_PUBLISH_PARAM, AUTOFF_INIT_TOLERANCE
from controller_overseer import SimulinkModelNode

class OverseerAutoTunePlugin:
    def __init__(self):
        pass
    
    
    def init_plugin(self, rosnode: Node, config, model: SimulinkModelNode):
        self.waiting_on_init = False
        self.drag_comp_forward_data = None
        self.drag_comp_reverse_data = None
        self.refresh_drag_file = True
        
        self.rosnode = rosnode
        self.model = model
        
        self.currentAutoTuneTwist = [0,0,0,0,0,0]
        self.autoff_config_path = None
        
        self.default_ff = config["controller"]["autoff"]["initial_ff"]
        
        #params
        rosnode.declare_parameter(FF_PUBLISH_PARAM, False)
        
        rosnode.declare_parameter("write_ff_autotune", True)
        self.writeFFAutoTune = rosnode.get_parameter("write_ff_autotune").value
        
        #pubs and subs
        self.re_init_signal_pub = rosnode.create_publisher(Empty, AUTOTUNE_REINIT_TOPIC_NAME, qos_profile_system_default)
        
        #sub to the ff auto tune results
        rosnode.create_subscription(Twist, "ff_auto_tune", self.ffAutoTuneCB, qos_profile_system_default)
        
        #sub to the drag compensator forward
        rosnode.create_subscription(TwistWithCovariance, "controller/drag_comp/forward", self.dragCompForwardCb, qos_profile_system_default)

        #sub to the drag compensator reverse
        rosnode.create_subscription(TwistWithCovariance, "controller/drag_comp/reverse", self.dragCompRevereseCb, qos_profile_system_default)
    
    
    def ffAutoTuneCB(self, msg: Twist):
        #write the autotune save if it has changed

        #if writing is disabled
        if not self.writeFFAutoTune:
            return
        
        #if the overseer has not initialized the controller yet
        if not self.model.loaded_params:
            return
    
        #if the auto ff config path doesn't exist
        if (self.autoff_config_path == None):
            return 

        #if the controller is trying to reinit the autoff
        if(self.waiting_on_init):

            auto_ff_init = None
            try:
                auto_ff_init = self.default_ff
            except KeyError:
                self.rosnode.get_logger().warn("Cannot find the initial auto ff in config tree!")
                return

            if (abs(auto_ff_init[0] - msg.linear.x) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[1] - msg.linear.y) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[2] - msg.linear.z) < AUTOFF_INIT_TOLERANCE and 
                abs(auto_ff_init[3] - msg.angular.x) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[4] - msg.angular.y) < AUTOFF_INIT_TOLERANCE and abs(auto_ff_init[5] - msg.angular.z) < AUTOFF_INIT_TOLERANCE):

                #if the init signal has taken
                self.waiting_on_init = False

            else: 
                #pub msg
                msg = Empty()
                self.re_init_signal_pub.publish(msg)

            return

        #if the twist has been updated
        if (not (self.currentAutoTuneTwist[0] == msg.linear.x and self.currentAutoTuneTwist[1] == msg.linear.y and self.currentAutoTuneTwist[2] == msg.linear.z and 
           self.currentAutoTuneTwist[3] == msg.angular.x and self.currentAutoTuneTwist[4] == msg.angular.y and self.currentAutoTuneTwist[5] == msg.angular.z)) or (self.refresh_drag_file):
            #prepare string to be wrote
            auto_ff_config_string = f"auto_ff: [{msg.linear.x},{msg.linear.y},{msg.linear.z},{msg.angular.x},{msg.angular.y},{msg.angular.z}]\n"
            
            drag_forward_string = ""
            drag_reverse_string = ""
            if(not ((self.drag_comp_forward_data is None) or (self.drag_comp_reverse_data is None))):

                #write the forward drag string
                drag_forward_string = f"drag_forward: ["
                for value in self.drag_comp_forward_data:
                    drag_forward_string = drag_forward_string + str(value) + ","
                drag_forward_string =  drag_forward_string + "]\n"

                #write the reverse drag string
                drag_reverse_string = f"drag_reverse: ["
                for value in self.drag_comp_reverse_data:
                    drag_reverse_string = drag_reverse_string + str(value) + ","
                drag_reverse_string =  drag_reverse_string + "]\n"

            #write config to file
            try:
                with open(self.autoff_config_path, "w") as config:
                    
                    config.write(auto_ff_config_string)
                    

                    if not ((self.drag_comp_forward_data is None) or (self.drag_comp_reverse_data is None)):
                     #cant save until cb complete
                        config.write(drag_forward_string)
                        config.write(drag_reverse_string)
                        
                    config.close()

            except FileExistsError:
                self.rosnode.get_logger().error(f"Cannot open ff auto tune file at: {self.autoff_config_path}")
            except PermissionError:
                self.rosnode.get_logger().error(f"No Permission to write of autoff file at: {self.autoff_config_path}")

    def dragCompForwardCb(self, msg: TwistWithCovariance):
        #save drag data
        self.drag_comp_forward_data = msg.covariance
        self.refresh_drag_file = True


    def dragCompRevereseCb(self, msg: TwistWithCovariance):
        #save drag data
        self.drag_comp_reverse_data = msg.covariance
        self.refresh_drag_file = True

def get_plugin():
    return OverseerAutoTunePlugin()
