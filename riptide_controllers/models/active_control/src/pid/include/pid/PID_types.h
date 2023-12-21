//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID_types.h
//
// Code generated for Simulink model 'PID'.
//
// Model version                  : 2.65
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Thu Dec 21 11:19:06 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_PID_types_h_
#define RTW_HEADER_PID_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
struct SL_Bus_geometry_msgs_Vector3
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Twist_

// MsgType=geometry_msgs/Twist
struct SL_Bus_geometry_msgs_Twist
{
  // MsgType=geometry_msgs/Vector3
  SL_Bus_geometry_msgs_Vector3 linear;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_geometry_msgs_Vector3 angular;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_

// MsgType=builtin_interfaces/Time
struct SL_Bus_builtin_interfaces_Time
{
  int32_T sec;
  uint32_T nanosec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_

// MsgType=std_msgs/Header
struct SL_Bus_std_msgs_Header
{
  // MsgType=builtin_interfaces/Time
  SL_Bus_builtin_interfaces_Time stamp;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=frame_id_SL_Info:TruncateAction=warn 
  uint8_T frame_id[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=frame_id
  SL_Bus_ROSVariableLengthArrayInfo frame_id_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
struct SL_Bus_geometry_msgs_Point
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_

// MsgType=geometry_msgs/Quaternion
struct SL_Bus_geometry_msgs_Quaternion
{
  real_T x;
  real_T y;
  real_T z;
  real_T w;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_

// MsgType=geometry_msgs/Pose
struct SL_Bus_geometry_msgs_Pose
{
  // MsgType=geometry_msgs/Point
  SL_Bus_geometry_msgs_Point position;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_geometry_msgs_Quaternion orientation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_PoseWithCovariance_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_PoseWithCovariance_

// MsgType=geometry_msgs/PoseWithCovariance
struct SL_Bus_geometry_msgs_PoseWithCovariance
{
  // MsgType=geometry_msgs/Pose
  SL_Bus_geometry_msgs_Pose pose;
  real_T covariance[36];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_TwistWithCovariance_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_TwistWithCovariance_

// MsgType=geometry_msgs/TwistWithCovariance
struct SL_Bus_geometry_msgs_TwistWithCovariance
{
  // MsgType=geometry_msgs/Twist
  SL_Bus_geometry_msgs_Twist twist;
  real_T covariance[36];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav_msgs_Odometry_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav_msgs_Odometry_

// MsgType=nav_msgs/Odometry
struct SL_Bus_nav_msgs_Odometry
{
  // MsgType=std_msgs/Header
  SL_Bus_std_msgs_Header header;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=child_frame_id_SL_Info:TruncateAction=warn 
  uint8_T child_frame_id[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=child_frame_id
  SL_Bus_ROSVariableLengthArrayInfo child_frame_id_SL_Info;

  // MsgType=geometry_msgs/PoseWithCovariance
  SL_Bus_geometry_msgs_PoseWithCovariance pose;

  // MsgType=geometry_msgs/TwistWithCovariance
  SL_Bus_geometry_msgs_TwistWithCovariance twist;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_

// MsgType=std_msgs/Bool
struct SL_Bus_std_msgs_Bool
{
  boolean_T data;
};

#endif

#ifndef struct_f_robotics_slcore_internal_bl_T
#define struct_f_robotics_slcore_internal_bl_T

struct f_robotics_slcore_internal_bl_T
{
  int32_T __dummy;
};

#endif                                // struct_f_robotics_slcore_internal_bl_T

#ifndef struct_ros_slros2_internal_block_Cur_T
#define struct_ros_slros2_internal_block_Cur_T

struct ros_slros2_internal_block_Cur_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  f_robotics_slcore_internal_bl_T SampleTimeHandler;
};

#endif                                // struct_ros_slros2_internal_block_Cur_T

#ifndef struct_ros_slros2_internal_block_Get_T
#define struct_ros_slros2_internal_block_Get_T

struct ros_slros2_internal_block_Get_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  f_robotics_slcore_internal_bl_T SampleTimeHandler;
};

#endif                                // struct_ros_slros2_internal_block_Get_T

// Custom Type definition for MATLABSystem: '<S8>/SourceBlock'
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "rmw/types.h"
#include "rmw/types.h"
#ifndef struct_ros_slros2_internal_block_Pub_T
#define struct_ros_slros2_internal_block_Pub_T

struct ros_slros2_internal_block_Pub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                // struct_ros_slros2_internal_block_Pub_T

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                // struct_ros_slros2_internal_block_Sub_T

// Parameters for system: '<Root>/Tranfsform Gains1'
typedef struct P_TranfsformGains1_PID_T_ P_TranfsformGains1_PID_T;

// Parameters (default storage)
typedef struct P_PID_T_ P_PID_T;

// Forward declaration for rtModel
typedef struct tag_RTM_PID_T RT_MODEL_PID_T;

#endif                                 // RTW_HEADER_PID_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
