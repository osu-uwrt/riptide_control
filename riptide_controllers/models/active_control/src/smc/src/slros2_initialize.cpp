// Copyright 2022 The MathWorks, Inc.
// Generated 19-Nov-2023 17:55:20
#include "slros2_initialize.h"
// SMC/Publish Active Control Forces
SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_SMC_179;
// SMC/Publish Curve Data/Publish
SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_SMC_252;
// SMC/Publish Curve Data/Publish1
SimulinkPublisher<std_msgs::msg::Float32,SL_Bus_std_msgs_Float32> Pub_SMC_257;
// SMC/Odometry Sub
SimulinkSubscriber<nav_msgs::msg::Odometry,SL_Bus_nav_msgs_Odometry> Sub_SMC_175;
// SMC/Set Point Sub
SimulinkSubscriber<geometry_msgs::msg::Pose,SL_Bus_geometry_msgs_Pose> Sub_SMC_171;
// For Block SMC/Eta 0
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_204;
// For Block SMC/Eta 1
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_206;
// For Block SMC/Scaling Parameter
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_208;
// For Block SMC/aMax
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_242;
// For Block SMC/jMax
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_244;
// For Block SMC/mass
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_190;
// For Block SMC/mass1
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_192;
// For Block SMC/mass2
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_193;
// For Block SMC/mass3
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_197;
// For Block SMC/mass4
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_199;
// For Block SMC/mass5
SimulinkParameterArrayGetter<int64_T,std::vector<int64_t>> ParamGet_SMC_202;
// For Block SMC/vMax
SimulinkParameterGetter<int64_T,int64_t> ParamGet_SMC_240;
