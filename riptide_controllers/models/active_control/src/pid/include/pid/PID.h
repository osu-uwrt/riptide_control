//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID.h
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
#ifndef RTW_HEADER_PID_h_
#define RTW_HEADER_PID_h_
#include "rtwtypes.h"
#include "slros2_initialize.h"
#include "PID_types.h"

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{

#include "rt_nonfinite.h"

}

#include <stddef.h>
#include "zero_crossing_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Block signals for system '<Root>/Tranfsform Gains1'
struct B_TranfsformGains1_PID_T {
  real_T TmpSignalConversionAtDotP_e[3];
  real_T Abs1[3];                      // '<S28>/Abs1'
  real_T TmpSignalConversionAtDot_et[3];
  real_T Zero;                         // '<S24>/Zero'
  real_T AccountNearInfiniteTangent;   // '<S24>/Account Near Infinite Tangent'
  real_T tanalpha2;                    // '<S24>/tan(alpha)^2'
  real_T AccountNearInfiniteTangent1; // '<S24>/Account Near Infinite Tangent1'
  real_T NumZ;                         // '<S24>/Num Z'
  real_T Zero_f;                       // '<S26>/Zero'
  real_T AccountNearInfiniteTangent_k; // '<S26>/Account Near Infinite Tangent'
  real_T tanalpha2_f;                  // '<S26>/tan(alpha)^2'
  real_T AccountNearInfiniteTangent1_e;
                                      // '<S26>/Account Near Infinite Tangent1'
  real_T NumZ_g;                       // '<S26>/Num Z'
  real_T Zero_h;                       // '<S28>/Zero'
  real_T AccountNearInfiniteTangent_m; // '<S28>/Account Near Infinite Tangent'
  real_T tanalpha2_b;                  // '<S28>/tan(alpha)^2'
  real_T AccountNearInfiniteTangent1_n;
                                      // '<S28>/Account Near Infinite Tangent1'
  real_T NumZ_gv;                      // '<S28>/Num Z'
  real_T YinBody;                      // '<S27>/Y in Body'
  real_T Sqrt1;                        // '<S28>/Sqrt1'
  real_T XTerm;                        // '<S23>/X Term'
};

// Block signals (default storage)
struct B_PID_T {
  SL_Bus_nav_msgs_Odometry In1;        // '<S20>/In1'
  SL_Bus_nav_msgs_Odometry b_varargout_2;
  real_T IdentityMatrix[9];            // '<Root>/IdentityMatrix'
  real_T world2BodyRotm[9];            // '<Root>/MATLAB Function'
  real_T Body2World[9];                // '<Root>/Body 2 World'
  real_T Transpose[9];                 // '<Root>/Transpose'
  SL_Bus_geometry_msgs_Pose In1_c;     // '<S21>/In1'
  SL_Bus_geometry_msgs_Pose b_varargout_2_m;
  SL_Bus_geometry_msgs_Twist BusAssignment;// '<S5>/Bus Assignment'
  real_T ControlSum[6];                // '<S2>/Control Sum'
  real_T Divide4[6];                   // '<Root>/Divide4'
  real_T IfSurfaceGains1[6];           // '<S9>/If Surface Gains1'
  real_T IfSurfaceGains[6];            // '<S9>/If Surface Gains'
  real_T normalizedError[6];           // '<S2>/Normalize Error'
  real_T rtb_WorldGain_p3_c[6];
  real_T rtb_Body2World_k[6];
  int64_T b_value[6];
  int64_T b_value_c[6];
  int64_T varargin_1[6];
  char_T b_zeroDelimTopic[36];
  char_T b_zeroDelimTopic_b[33];
  real_T W2B[4];
  real_T W2B_p[4];
  real_T rtb_TmpSignalConversionAtSFun_c[4];
  char_T b_zeroDelimTopic_f[27];
  char_T b_zeroDelimTopic_g[25];
  char_T prmName[24];
  real_T axisError[3];
  char_T prmName_g[23];
  char_T prmName_m[20];
  char_T prmName_n[16];
  real_T a;
  real_T absxk;
  real_T t;
  real_T WorldGain_g;                  // '<S35>/World Gain'
  real_T WorldGain_m;                  // '<S37>/World Gain'
  real_T WorldGain_p;                  // '<S39>/World Gain'
  real_T WorldGain_a;                  // '<S40>/World Gain'
  real_T IfForceSurfaceGains;          // '<S9>/If Force Surface Gains'
  real_T Divide8;                      // '<Root>/Divide8'
  real_T WorldGain_e;                  // '<S29>/World Gain'
  real_T WorldGain_d;                  // '<S31>/World Gain'
  real_T WorldGain_eo;                 // '<S32>/World Gain'
  real_T WorldGain_pm;                 // '<S34>/World Gain'
  real_T WorldGain_p3;                 // '<S23>/World Gain'
  real_T WorldGain_g2;                 // '<S25>/World Gain'
  real_T WorldGain_l;                  // '<S27>/World Gain'
  real_T WorldGain_ad;                 // '<S24>/World Gain'
  real_T WorldGain_c;                  // '<S26>/World Gain'
  real_T WorldGain_px;                 // '<S28>/World Gain'
  real_T Divide;                       // '<Root>/Divide'
  real_T q_idx_0;
  real_T q_idx_2;
  real_T q_idx_3;
  real_T q_idx_1;
  SL_Bus_std_msgs_Bool In1_p;          // '<S22>/In1'
  int64_T b_value_p;
  int64_T b_value_l;
  int32_T i;
  int32_T rtb_world2BodyRotm_tmp;
  int32_T k;
  uint32_T len;
  boolean_T haveGainsChanged;          // '<S1>/Check if gains change'
  B_TranfsformGains1_PID_T TranfsformGains3;// '<Root>/Tranfsform Gains3'
  B_TranfsformGains1_PID_T TranfsformGains2;// '<Root>/Tranfsform Gains2'
  B_TranfsformGains1_PID_T TranfsformGains1;// '<Root>/Tranfsform Gains1'
};

// Block states (default storage) for system '<Root>'
struct DW_PID_T {
  ros_slros2_internal_block_Cur_T obj; // '<Root>/Current Time'
  ros_slros2_internal_block_Get_T obj_c;// '<Root>/surface gain floor'
  ros_slros2_internal_block_Get_T obj_h;// '<Root>/surface gain buffer'
  ros_slros2_internal_block_Get_T obj_b;// '<Root>/p surface'
  ros_slros2_internal_block_Get_T obj_g;// '<Root>/p Gains'
  ros_slros2_internal_block_Get_T obj_bm;// '<Root>/max_control'
  ros_slros2_internal_block_Get_T obj_bv;// '<Root>/i surface'
  ros_slros2_internal_block_Get_T obj_n;// '<Root>/i Gains'
  ros_slros2_internal_block_Get_T obj_a;// '<Root>/d surface'
  ros_slros2_internal_block_Get_T obj_e;// '<Root>/d Gains'
  ros_slros2_internal_block_Get_T obj_j;// '<Root>/Get Parameter1'
  ros_slros2_internal_block_Get_T obj_d;// '<Root>/Get Parameter'
  ros_slros2_internal_block_Pub_T obj_ng;// '<S19>/SinkBlock'
  ros_slros2_internal_block_Sub_T obj_av;// '<S8>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_k;// '<S7>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_ex;// '<S6>/SourceBlock'
  real_T PreviousGainState_PreviousInput;// '<S9>/Previous Gain State'
  real_T AccumulatedError_PreviousInput[6];// '<S2>/Accumulated Error'
  real_T PreviousTime_PreviousInput;   // '<S2>/Previous Time'
  real_T PreviousP_PreviousInput[6];   // '<S1>/Previous P'
  real_T PreviousI_PreviousInput[6];   // '<S1>/Previous I'
  real_T PreviousD_PreviousInput[6];   // '<S1>/Previous D'
};

// Zero-crossing (trigger) state
struct PrevZCX_PID_T {
  ZCSigState PublishControl_Trig_ZCE;  // '<Root>/Publish Control'
  ZCSigState ControlSystem_Trig_ZCE;   // '<Root>/Control System'
  ZCSigState CheckifGainsHaveChanged_Trig_ZC;// '<Root>/Check if Gains Have Changed' 
};

// Parameters for system: '<Root>/Tranfsform Gains1'
struct P_TranfsformGains1_PID_T_ {
  real_T WorldPrincipalX_Value[3];     // Expression: [1;0;0]
                                          //  Referenced by: '<S10>/World Principal X'

  real_T Constant1_Value;              // Expression: -1/2
                                          //  Referenced by: '<S23>/Constant1'

  real_T Constant2_Value;              // Expression: 1
                                          //  Referenced by: '<S23>/Constant2'

  real_T Infinity_Value;               // Expression: 999999
                                          //  Referenced by: '<S23>/Infinity'

  real_T NonZeroGains_Threshold;       // Expression: 0
                                          //  Referenced by: '<S23>/Non Zero Gains'

  real_T Constant_Value;               // Expression: 1
                                          //  Referenced by: '<S23>/Constant'

  real_T NonZeroGains1_Threshold;      // Expression: 0
                                          //  Referenced by: '<S23>/Non Zero Gains 1'

  real_T StopInfiniteTangent_Threshold;// Expression: 0
                                          //  Referenced by: '<S23>/Stop Infinite Tangent'

  real_T NonZeroGains2_Threshold;      // Expression: 0
                                          //  Referenced by: '<S23>/Non Zero Gains 2'

  real_T StopInfiniteTanget2_Threshold;// Expression: 0
                                          //  Referenced by: '<S23>/Stop Infinite Tanget 2'

  real_T Constant1_Value_i;            // Expression: -1/2
                                          //  Referenced by: '<S24>/Constant1'

  real_T Constant2_Value_b;            // Expression: 1
                                          //  Referenced by: '<S24>/Constant2'

  real_T Infinity_Value_d;             // Expression: 999999
                                          //  Referenced by: '<S24>/Infinity'

  real_T NonZeroGains_Threshold_n;     // Expression: 0
                                          //  Referenced by: '<S24>/Non Zero Gains'

  real_T Constant_Value_l;             // Expression: 1
                                          //  Referenced by: '<S24>/Constant'

  real_T NonZeroGains1_Threshold_p;    // Expression: 0
                                          //  Referenced by: '<S24>/Non Zero Gains 1'

  real_T StopInfiniteTangent_Threshold_a;// Expression: 0
                                            //  Referenced by: '<S24>/Stop Infinite Tangent'

  real_T NonZeroGains2_Threshold_k;    // Expression: 0
                                          //  Referenced by: '<S24>/Non Zero Gains 2'

  real_T StopInfiniteTanget2_Threshold_l;// Expression: 0
                                            //  Referenced by: '<S24>/Stop Infinite Tanget 2'

  real_T WorldPrincipalY_Value[3];     // Expression: [0;1;0]
                                          //  Referenced by: '<S10>/World Principal Y'

  real_T Constant1_Value_l;            // Expression: -1/2
                                          //  Referenced by: '<S25>/Constant1'

  real_T Constant2_Value_f;            // Expression: 1
                                          //  Referenced by: '<S25>/Constant2'

  real_T Infinity_Value_c;             // Expression: 999999
                                          //  Referenced by: '<S25>/Infinity'

  real_T NonZeroGains_Threshold_d;     // Expression: 0
                                          //  Referenced by: '<S25>/Non Zero Gains'

  real_T Constant_Value_k;             // Expression: 1
                                          //  Referenced by: '<S25>/Constant'

  real_T NonZeroGains1_Threshold_m;    // Expression: 0
                                          //  Referenced by: '<S25>/Non Zero Gains 1'

  real_T StopInfiniteTangent_Threshol_au;// Expression: 0
                                            //  Referenced by: '<S25>/Stop Infinite Tangent'

  real_T NonZeroGains2_Threshold_o;    // Expression: 0
                                          //  Referenced by: '<S25>/Non Zero Gains 2'

  real_T StopInfiniteTanget2_Threshold_p;// Expression: 0
                                            //  Referenced by: '<S25>/Stop Infinite Tanget 2'

  real_T Constant1_Value_n;            // Expression: -1/2
                                          //  Referenced by: '<S26>/Constant1'

  real_T Constant2_Value_d;            // Expression: 1
                                          //  Referenced by: '<S26>/Constant2'

  real_T Infinity_Value_e;             // Expression: 999999
                                          //  Referenced by: '<S26>/Infinity'

  real_T NonZeroGains_Threshold_k;     // Expression: 0
                                          //  Referenced by: '<S26>/Non Zero Gains'

  real_T Constant_Value_g;             // Expression: 1
                                          //  Referenced by: '<S26>/Constant'

  real_T NonZeroGains1_Threshold_d;    // Expression: 0
                                          //  Referenced by: '<S26>/Non Zero Gains 1'

  real_T StopInfiniteTangent_Threshold_i;// Expression: 0
                                            //  Referenced by: '<S26>/Stop Infinite Tangent'

  real_T NonZeroGains2_Threshold_p;    // Expression: 0
                                          //  Referenced by: '<S26>/Non Zero Gains 2'

  real_T StopInfiniteTanget2_Threshold_g;// Expression: 0
                                            //  Referenced by: '<S26>/Stop Infinite Tanget 2'

  real_T WorldPrincipalZ_Value[3];     // Expression: [0;0;1]
                                          //  Referenced by: '<S10>/World Principal Z'

  real_T Constant1_Value_k;            // Expression: -1/2
                                          //  Referenced by: '<S27>/Constant1'

  real_T Constant2_Value_k;            // Expression: 1
                                          //  Referenced by: '<S27>/Constant2'

  real_T Infinity_Value_h;             // Expression: 999999
                                          //  Referenced by: '<S27>/Infinity'

  real_T NonZeroGains_Threshold_o;     // Expression: 0
                                          //  Referenced by: '<S27>/Non Zero Gains'

  real_T Constant_Value_m;             // Expression: 1
                                          //  Referenced by: '<S27>/Constant'

  real_T NonZeroGains1_Threshold_f;    // Expression: 0
                                          //  Referenced by: '<S27>/Non Zero Gains 1'

  real_T StopInfiniteTangent_Threshold_n;// Expression: 0
                                            //  Referenced by: '<S27>/Stop Infinite Tangent'

  real_T NonZeroGains2_Threshold_kl;   // Expression: 0
                                          //  Referenced by: '<S27>/Non Zero Gains 2'

  real_T StopInfiniteTanget2_Threshold_a;// Expression: 0
                                            //  Referenced by: '<S27>/Stop Infinite Tanget 2'

  real_T Constant1_Value_e;            // Expression: -1/2
                                          //  Referenced by: '<S28>/Constant1'

  real_T Constant2_Value_g;            // Expression: 1
                                          //  Referenced by: '<S28>/Constant2'

  real_T Infinity_Value_d4;            // Expression: 999999
                                          //  Referenced by: '<S28>/Infinity'

  real_T NonZeroGains_Threshold_f;     // Expression: 0
                                          //  Referenced by: '<S28>/Non Zero Gains'

  real_T Constant_Value_d;             // Expression: 1
                                          //  Referenced by: '<S28>/Constant'

  real_T NonZeroGains1_Threshold_b;    // Expression: 0
                                          //  Referenced by: '<S28>/Non Zero Gains 1'

  real_T StopInfiniteTangent_Threshold_m;// Expression: 0
                                            //  Referenced by: '<S28>/Stop Infinite Tangent'

  real_T NonZeroGains2_Threshold_a;    // Expression: 0
                                          //  Referenced by: '<S28>/Non Zero Gains 2'

  real_T StopInfiniteTanget2_Threshol_gv;// Expression: 0
                                            //  Referenced by: '<S28>/Stop Infinite Tanget 2'

};

// Parameters (default storage)
struct P_PID_T_ {
  SL_Bus_nav_msgs_Odometry Out1_Y0;    // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S20>/Out1'

  SL_Bus_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                             //  Referenced by: '<S6>/Constant'

  SL_Bus_geometry_msgs_Pose Out1_Y0_d; // Computed Parameter: Out1_Y0_d
                                          //  Referenced by: '<S21>/Out1'

  SL_Bus_geometry_msgs_Pose Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                //  Referenced by: '<S7>/Constant'

  SL_Bus_geometry_msgs_Twist Constant_Value_m;// Computed Parameter: Constant_Value_m
                                                 //  Referenced by: '<S18>/Constant'

  SL_Bus_std_msgs_Bool Out1_Y0_i;      // Computed Parameter: Out1_Y0_i
                                          //  Referenced by: '<S22>/Out1'

  SL_Bus_std_msgs_Bool Constant_Value_h;// Computed Parameter: Constant_Value_h
                                           //  Referenced by: '<S8>/Constant'

  real_T PreviousP_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S1>/Previous P'

  real_T PreviousI_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S1>/Previous I'

  real_T PreviousD_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S1>/Previous D'

  real_T Control_Y0;                   // Computed Parameter: Control_Y0
                                          //  Referenced by: '<S2>/Control'

  real_T AccumulatedError_InitialConditi;// Expression: 0
                                            //  Referenced by: '<S2>/Accumulated Error'

  real_T PreviousTime_InitialCondition;// Expression: -1
                                          //  Referenced by: '<S2>/Previous Time'

  real_T IControl_Gain;                // Expression: -1
                                          //  Referenced by: '<S2>/I Control'

  real_T PControl_Gain;                // Expression: -1
                                          //  Referenced by: '<S2>/P Control'

  real_T DControl_Gain;                // Expression: -1
                                          //  Referenced by: '<S2>/D Control'

  real_T Constant_Value_mr;            // Expression: 0
                                          //  Referenced by: '<S9>/Constant'

  real_T Constant1_Value;              // Expression: 1
                                          //  Referenced by: '<S9>/Constant1'

  real_T IdentityMatrix_IDMatrixData[9];
                              // Computed Parameter: IdentityMatrix_IDMatrixData
                                 //  Referenced by: '<Root>/IdentityMatrix'

  real_T PreviousGainState_InitialCondit;// Expression: 0
                                            //  Referenced by: '<S9>/Previous Gain State'

  real_T IfForceDefaultGains_Threshold;// Expression: 0
                                          //  Referenced by: '<S9>/If Force Default Gains'

  real_T IfForceSurfaceGains_Threshold;// Expression: 0
                                          //  Referenced by: '<S9>/If Force Surface Gains'

  real_T IfSurfaceGains2_Threshold;    // Expression: 0
                                          //  Referenced by: '<S9>/If Surface Gains2'

  real_T IfSurfaceGains1_Threshold;    // Expression: 0
                                          //  Referenced by: '<S9>/If Surface Gains1'

  real_T IfSurfaceGains_Threshold;     // Expression: 0
                                          //  Referenced by: '<S9>/If Surface Gains'

  real_T Gain_Gain;                    // Expression: -1
                                          //  Referenced by: '<S3>/Gain'

  boolean_T haveGainsChanged_Y0;      // Computed Parameter: haveGainsChanged_Y0
                                         //  Referenced by: '<S1>/haveGainsChanged'

  P_TranfsformGains1_PID_T TranfsformGains3;// '<Root>/Tranfsform Gains3'
  P_TranfsformGains1_PID_T TranfsformGains2;// '<Root>/Tranfsform Gains2'
  P_TranfsformGains1_PID_T TranfsformGains1;// '<Root>/Tranfsform Gains1'
};

// Real-time Model Data Structure
struct tag_RTM_PID_T {
  const char_T * volatile errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

// Class declaration for model PID
class PID
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_PID_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  PID();

  // Destructor
  ~PID();

  // private data and function members
 private:
  // Block signals
  B_PID_T PID_B;

  // Block states
  DW_PID_T PID_DW;

  // Tunable parameters
  static P_PID_T PID_P;

  // Triggered events
  PrevZCX_PID_T PID_PrevZCX;

  // private member function(s) for subsystem '<Root>/Tranfsform Gains1'
  void PID_TranfsformGains1(const real_T rtu_TwistBody[6], const real_T
    rtu_LinearRotm[9], const real_T rtu_AngularRotm[9], real_T
    *rty_WorldFrameGains, real_T *rty_WorldFrameGains_i, real_T
    *rty_WorldFrameGains_f, real_T *rty_WorldFrameGains_o, real_T
    *rty_WorldFrameGains_on, real_T *rty_WorldFrameGains_c,
    B_TranfsformGains1_PID_T *localB, P_TranfsformGains1_PID_T *localP);

  // private member function(s) for subsystem '<Root>'
  boolean_T PID_isequal(const real_T varargin_1[6], const real_T varargin_2[6]);
  void PID_quatmultiply(const real_T q[4], const real_T r[4], real_T qout[4]);
  void PID_SystemCore_setup(ros_slros2_internal_block_Pub_T *obj);
  void PID_SystemCore_setup_n(ros_slros2_internal_block_Sub_T *obj);
  void PID_SystemCore_setup_nt(ros_slros2_internal_block_Sub_T *obj);
  void PID_SystemCore_setup_ntv(ros_slros2_internal_block_Sub_T *obj);

  // Real-Time Model
  RT_MODEL_PID_T PID_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PID'
//  '<S1>'   : 'PID/Check if Gains Have Changed'
//  '<S2>'   : 'PID/Control System'
//  '<S3>'   : 'PID/Enforce Max Control Constraints'
//  '<S4>'   : 'PID/MATLAB Function'
//  '<S5>'   : 'PID/Publish Control'
//  '<S6>'   : 'PID/Subscribe'
//  '<S7>'   : 'PID/Subscribe1'
//  '<S8>'   : 'PID/Subscribe2'
//  '<S9>'   : 'PID/Switch Submerdge Gains'
//  '<S10>'  : 'PID/Tranfsform Gains1'
//  '<S11>'  : 'PID/Tranfsform Gains2'
//  '<S12>'  : 'PID/Tranfsform Gains3'
//  '<S13>'  : 'PID/Twist Body2World4'
//  '<S14>'  : 'PID/Twist World 2 Body'
//  '<S15>'  : 'PID/Check if Gains Have Changed/Check if gains change'
//  '<S16>'  : 'PID/Control System/Manage Accumulated Error'
//  '<S17>'  : 'PID/Control System/Normalize Error'
//  '<S18>'  : 'PID/Publish Control/Blank Message'
//  '<S19>'  : 'PID/Publish Control/Publish Active Control'
//  '<S20>'  : 'PID/Subscribe/Enabled Subsystem'
//  '<S21>'  : 'PID/Subscribe1/Enabled Subsystem'
//  '<S22>'  : 'PID/Subscribe2/Enabled Subsystem'
//  '<S23>'  : 'PID/Tranfsform Gains1/Get World Frame Gain X'
//  '<S24>'  : 'PID/Tranfsform Gains1/Get World Frame Gain X1'
//  '<S25>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y'
//  '<S26>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y1'
//  '<S27>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Z'
//  '<S28>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Z1'
//  '<S29>'  : 'PID/Tranfsform Gains2/Get World Frame Gain X'
//  '<S30>'  : 'PID/Tranfsform Gains2/Get World Frame Gain X1'
//  '<S31>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y'
//  '<S32>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y1'
//  '<S33>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Z'
//  '<S34>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Z1'
//  '<S35>'  : 'PID/Tranfsform Gains3/Get World Frame Gain X'
//  '<S36>'  : 'PID/Tranfsform Gains3/Get World Frame Gain X1'
//  '<S37>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y'
//  '<S38>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y1'
//  '<S39>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Z'
//  '<S40>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Z1'

#endif                                 // RTW_HEADER_PID_h_

//
// File trailer for generated code.
//
// [EOF]
//
