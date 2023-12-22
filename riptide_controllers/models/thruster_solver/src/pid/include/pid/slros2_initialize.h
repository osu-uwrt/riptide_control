// Copyright 2022 The MathWorks, Inc.
// Generated 22-Dec-2023 14:00:05
#ifndef _SLROS2_INITIALIZE_H_
#define _SLROS2_INITIALIZE_H_
#include "PID_types.h"
// Generic pub-sub header
#include "slros2_generic_pubsub.h"
// Generic service header
#include "slros2_generic_service.h"
#include "slros2_time.h"
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
// PID/Publish Control/Publish Active Control
extern SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_PID_53;
// PID/Subscribe
extern SimulinkSubscriber<nav_msgs::msg::Odometry,SL_Bus_nav_msgs_Odometry> Sub_PID_33;
// PID/Subscribe1
extern SimulinkSubscriber<geometry_msgs::msg::Pose,SL_Bus_geometry_msgs_Pose> Sub_PID_39;
// PID/Subscribe2
extern SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_PID_2028;
// For Block PID/Get Parameter
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_43;
// For Block PID/Get Parameter1
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_44;
// For Block PID/d Gains
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_47;
// For Block PID/d surface
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_233;
// For Block PID/i Gains
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_46;
// For Block PID/i surface
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_234;
// For Block PID/max_control
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_123;
// For Block PID/p Gains
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_48;
// For Block PID/p surface
extern SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_235;
// For Block PID/surface gain buffer
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_239;
// For Block PID/surface gain floor
extern SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_240;
#endif
