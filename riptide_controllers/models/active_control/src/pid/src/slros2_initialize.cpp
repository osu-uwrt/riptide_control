// Copyright 2022 The MathWorks, Inc.
// Generated 21-Dec-2023 11:19:09
#include "slros2_initialize.h"
// PID/Publish Control/Publish Active Control
SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_PID_53;
// PID/Subscribe
SimulinkSubscriber<nav_msgs::msg::Odometry,SL_Bus_nav_msgs_Odometry> Sub_PID_33;
// PID/Subscribe1
SimulinkSubscriber<geometry_msgs::msg::Pose,SL_Bus_geometry_msgs_Pose> Sub_PID_39;
// PID/Subscribe2
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_PID_2028;
// For Block PID/Get Parameter
SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_43;
// For Block PID/Get Parameter1
SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_44;
// For Block PID/d Gains
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_47;
// For Block PID/d surface
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_233;
// For Block PID/i Gains
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_46;
// For Block PID/i surface
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_234;
// For Block PID/max_control
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_123;
// For Block PID/p Gains
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_48;
// For Block PID/p surface
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_PID_235;
// For Block PID/surface gain buffer
SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_239;
// For Block PID/surface gain floor
SimulinkParameterGetter<int64_T,int64_t> ParamGet_PID_240;
