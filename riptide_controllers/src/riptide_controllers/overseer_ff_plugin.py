from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist

from overseer_global_defs import FF_TOPIC_NAME, FF_PUBLISH_PARAM
from controller_overseer import SimulinkModelNode

class OverseerFFPlugin:
    def __init__(self):
        pass
    
    
    def init_plugin(self, rosnode: Node, config, model: SimulinkModelNode):
        self.rosnode = rosnode
        
        self.base_wrench = config["controller"]["feed_forward"]["base_wrench"]
        
        self.ffPub = rosnode.create_publisher(Twist, FF_TOPIC_NAME, qos_profile_system_default)
        self.timer = rosnode.create_timer(1.0, self.update_ff)
        
    
    def update_ff(self):
        # publish ff if necessary
        if not (self.rosnode.get_parameter(FF_PUBLISH_PARAM).value):
            #publish the ff
            self.publishingFF = True

            msg = Twist()
            msg.linear.x = self.base_wrench[0]
            msg.linear.y = self.base_wrench[1]
            msg.linear.z = self.base_wrench[2]
            msg.angular.x = self.base_wrench[3]
            msg.angular.y = self.base_wrench[4]
            msg.angular.z = self.base_wrench[5]

            self.ffPub.publish(msg)
        elif self.publishingFF == True:
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

def get_plugin():
    return OverseerFFPlugin()
