// Copyright 2022 The MathWorks, Inc.
// Generated 19-Nov-2023 17:55:20
#ifndef _SLROS2_INITIALIZE_H_
#define _SLROS2_INITIALIZE_H_
#include "SMC_types.h"
// Generic pub-sub header
#include "slros2_generic_pubsub.h"
// Generic service header
#include "slros2_generic_service.h"
#include "slros2_generic_param.h"
extern rclcpp::Node::SharedPtr SLROSNodePtr;
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, hist, dep, dur, rel)  \
    {                                                   \
        qosStruct.history = hist;                       \
        qosStruct.depth = dep;                          \
        qosStruct.durability = dur;                     \
        qosStruct.reliability = rel;                    \
    }
#endif
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qosProfile));
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == qosProfile.durability) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == qosProfile.reliability) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    return qos;
}
// SMC/Publish Active Control Forces
extern SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_SMC_179;
// SMC/Publish Curve Data/Publish
extern SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_SMC_252;
// SMC/Publish Curve Data/Publish1
extern SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_SMC_257;
// SMC/Odometry Sub
extern SimulinkSubscriber<nav_msgs::msg::Odometry,SL_Bus_nav_msgs_Odometry> Sub_SMC_175;
// SMC/Set Point Sub
extern SimulinkSubscriber<geometry_msgs::msg::Pose,SL_Bus_geometry_msgs_Pose> Sub_SMC_171;
// For Block SMC/Eta 0
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_204;
// For Block SMC/Eta 1
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_206;
// For Block SMC/Scaling Parameter
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_208;
// For Block SMC/aMax
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_242;
// For Block SMC/jMax
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_244;
// For Block SMC/mass
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_190;
// For Block SMC/mass1
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_192;
// For Block SMC/mass2
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_193;
// For Block SMC/mass3
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_197;
// For Block SMC/mass4
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_199;
// For Block SMC/mass5
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_202;
// For Block SMC/vMax
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_240;
#endif
