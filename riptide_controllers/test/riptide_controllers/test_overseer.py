#! /usr/bin/env python3

#
# UWRT Controller Overseer test script.
#

import os
import rclpy
import yaml
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import ParameterValue, ParameterType
from rcl_interfaces.srv import ListParameters, GetParameters
from ros2node.api import get_node_names
from ament_index_python import get_package_share_directory

OVERSEER_SCALE_FACTOR = 1000000

#
# UTIL FUNCTIONS
#

# def get_node_name_containing_name(node: Node, name: str):
#     active_nodes = node.get_node_names_and_namespaces()
    
#     nodes_with_name = [node for node in active_nodes if name in node[0]]
#     num_nodes = len(nodes_with_name)
#     if num_nodes == 1:
#         full_name = nodes_with_name[0][1] + "/" + nodes_with_name[0][0]
#         if full_name.startswith('//'):
#             full_name = full_name[1:]
        
#         return full_name
    
#     node.get_logger().error(f"Detected {num_nodes} nodes containing the name {name}. Nodes: {active_nodes}")
    # return ""

def get_node_name_containing_name(node: Node, name: str):
    for i in range(0, 4):
        active_nodes = get_node_names(node=node)
        nodes_with_name = [node for node in active_nodes if name in node.name]
        num_nodes = len(nodes_with_name)
        if num_nodes == 1:
            full_name = nodes_with_name[0].namespace + "/" + nodes_with_name[0].name
            if full_name.startswith("//"):
                full_name = full_name[1:]
            
            return full_name
        
        node.get_logger().error(f"Detected {num_nodes} nodes containing the name {name}. Nodes: {active_nodes}")
    return ""


def read_robot_config_to_dict(node: Node):
    robot_name = node.get_namespace()
    if robot_name.startswith('/'):
        robot_name = robot_name[1:]
    
    if len(robot_name) < 1:
        node.get_logger().error("Robot name incorrectly detected as a blank string")
        return None
    
    config_file_path = os.path.join(
        get_package_share_directory("riptide_descriptions2"),
        "config", f"{robot_name}.yaml")
    
    if not os.path.exists(config_file_path):
        node.get_logger().error(f"Config file path {robot_name} does not exist!")
        return None

    d = None
    with open(config_file_path, 'r') as f:
        d = yaml.safe_load(f)
    
    return d


def call_service_sync(node: Node, srv_type: type, srv_name: str, request):
    client = node.create_client(srv_type, srv_name)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def lookup_param_in_config(config: dict, param_name: str):
    split_path = str.split(param_name, "__")
    try:
        for part_path in split_path:
            config = config[part_path]
    except:
        return None, None
    else:
        #executed when no exception
        return config, type(config)


def get_parameter_value(node: Node, param: ParameterValue):
    match param.type:
        case ParameterType.PARAMETER_BOOL:
            return param.bool_value
        case ParameterType.PARAMETER_INTEGER:
            return param.integer_value[0] / OVERSEER_SCALE_FACTOR
        case ParameterType.PARAMETER_DOUBLE:
            return param.double_value
        case ParameterType.PARAMETER_STRING:
            return param.string_value
        case ParameterType.PARAMETER_BYTE_ARRAY:
            return param.byte_array_value
        case ParameterType.PARAMETER_BOOL_ARRAY:
            return param.bool_array_value
        case ParameterType.PARAMETER_INTEGER_ARRAY:
            return [param.integer_array_value[i] / OVERSEER_SCALE_FACTOR for i in range(0, len(param.integer_array_value))]
        case ParameterType.PARAMETER_DOUBLE_ARRAY:
            return param.double_array_value
        case ParameterType.PARAMETER_STRING_ARRAY:
            return param.string_array_value
        case _:
            node.get_logger().error(f"Value of parameter {param} is None!")
            return None
    


#
# TEST FUNCTIONS
#

# tests the overseer parameter set 
def test_parameter_set(node: Node):
    # ensure overseer up
    overseer_name = get_node_name_containing_name(node, "controller_overseer")
    assert overseer_name != ""
    
    # ensure complete controller up and get its name
    complete_controller_name = get_node_name_containing_name(node, "complete_controller")
    assert complete_controller_name != ""
    
    # load config
    config = read_robot_config_to_dict(node)
    assert config is not None
    
    #list parameters from complete controller node
    list_params_result = call_service_sync(node, ListParameters, f"{complete_controller_name}/list_parameters", ListParameters.Request())
    
    #get the parameters from the complete controller node
    get_params_request = GetParameters.Request()
    get_params_request.names = list_params_result.result.names
    get_params_result = call_service_sync(node, GetParameters, f"{complete_controller_name}/get_parameters", get_params_request)
    
    for i in range(0, len(get_params_result.values)):
        param = get_params_result.values[i]
        param_name = list_params_result.result.names[i]
        
        config_value, t = lookup_param_in_config(config, param_name)
        if config_value is None:
            continue
        
        if t == int:
            config_value = float(config_value)
        
        param_value = get_parameter_value(node, param)
        values_match = param_value == config_value
        
        if not values_match:
            node.get_logger().error(f"Value {param_name} does not match; expected {config_value}, got {param_value}")
        
    
    return True


#
# MAIN
#
def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("overseer_tester")
    executor = SingleThreadedExecutor()
    
    #spin for a little bit to let ros catch up (TODO did this fix anything?)
    start_time = node.get_clock().now()
    current_time = start_time
    while rclpy.ok() and (current_time - start_time).to_msg().sec < 1:
        rclpy.spin_once(node, executor=executor, timeout_sec=0.01)
        current_time = node.get_clock().now()
        
    #test functions here
    node.get_logger().info("test_parameter_set() - RUNNING")
    test_parameter_set(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
