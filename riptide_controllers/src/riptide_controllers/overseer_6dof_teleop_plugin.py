from rclpy.node import Node
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

from overseer_global_defs import ACTIVE_PARAMETERS_MASK
from controller_overseer import SimulinkModelNode

class Overseer6DOFTeleopPlugin:
    def __init__(self):
        pass
    
    
    def init_plugin(self, rosnode: Node, config, model: SimulinkModelNode):
        self.rosnode = rosnode
        self.model = model
        self.setTeleop = rosnode.create_service(SetBool, "setTeleop", self.setTeleop)
    
    
    def setTeleop(self, request, future):
        #set the teleop mode

        #if setting Teleop on
        if(request.data == True):
            try:
                #set the control mask values
                pVal = ParameterValue()
                pVal.type = ParameterType.PARAMETER_INTEGER_ARRAY
                pVal.integer_array_value = [3000000, 3000000, 2000000, 1000000, 1000000, 3000000]

                param = Parameter()
                param.value = pVal
                param.name = ACTIVE_PARAMETERS_MASK

                request = SetParameters.Request()
                request.parameters = [param]

                self.model.set_param_client.schedule_call(request, self.model.set_parameters_done_callback)

                future.success = True
                future.message = "Successfully enabled teleop!"

            except Exception as E:

                if(type(E) == AttributeError):
                    #this is an expected error so elaborate
                    self.rosnode.get_logger().warn("Failed to initialize teleop, model is not started!")
                    future.success = False
                    future.message = "Failed to initialize teleop, model is not started!"
                else:
                    self.rosnode.get_logger().warn(f"Failed to initialize teleop! {E}")
                    future.success = False
                    future.message = f"Failed to initialize teleop! {E}" 
        else:
        #putting into active control
            try:
                #set the control mask values
                pVal = ParameterValue()
                pVal.type = ParameterType.PARAMETER_INTEGER_ARRAY
                pVal.integer_array_value = [2000000, 2000000, 2000000, 1000000, 1000000, 2000000]

                param = Parameter()
                param.value = pVal
                param.name = ACTIVE_PARAMETERS_MASK

                request = SetParameters.Request()
                request.parameters = [param]

                self.model.set_param_client.schedule_call(request, self.model.set_parameters_done_callback)

                future.success = True
                future.message = "Successfully enabled active control!"

            except Exception as E:

                if(type(E) == AttributeError):
                    #this is an expected error so elaborate
                    self.rosnode.get_logger().warn("Failed to initialize active control, model is not started!")
                    future.success = False
                    future.message = "Failed to initialize active control, model is not started!"
                else:
                    self.rosnode.get_logger().warn(f"Failed to initialize active control! {E}")
                    future.success = False
                    future.message = f"Failed to initialize active control! {E}" 
                
        return future

def get_plugin():
    return Overseer6DOFTeleopPlugin()
