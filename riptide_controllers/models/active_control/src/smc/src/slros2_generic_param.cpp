/* Copyright 2022 The MathWorks, Inc. */

#include "slros2_generic_param.h"

extern rclcpp::Node::SharedPtr SLROSNodePtr;
/**
 * Initialize the parameter getter class.
 * @param pName The name of the ROS parameter
 */
void SimulinkParameterGetterBase::initParam(const std::string& pName) {
    nodePtr = SLROSNodePtr;
    paramName = pName;
}