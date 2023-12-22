//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID.h
//
// Code generated for Simulink model 'PID'.
//
// Model version                  : 2.70
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Fri Dec 22 14:00:03 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objective: Execution efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_PID_h_
#define RTW_HEADER_PID_h_
#include "rtwtypes.h"
#include "slros2_initialize.h"
#include "PID_types.h"
#include "rt_zcfcn.h"

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
  real_T Abs1[3];                      // '<S24>/Abs1'
  real_T TmpSignalConversionAtDotP_e[3];
  real_T TmpSignalConversionAtDot_et[3];
  real_T AccountNearInfiniteTangent;   // '<S23>/Account Near Infinite Tangent'
  real_T tanalpha2;                    // '<S23>/tan(alpha)^2'
  real_T AccountNearInfiniteTangent1; // '<S23>/Account Near Infinite Tangent1'
  real_T NumZ;                         // '<S23>/Num Z'
  real_T AccountNearInfiniteTangent_c; // '<S25>/Account Near Infinite Tangent'
  real_T tanalpha2_o;                  // '<S25>/tan(alpha)^2'
  real_T AccountNearInfiniteTangent1_d;
                                      // '<S25>/Account Near Infinite Tangent1'
  real_T NumZ_f;                       // '<S25>/Num Z'
  real_T AccountNearInfiniteTangent_b; // '<S24>/Account Near Infinite Tangent'
  real_T tanalpha2_c;                  // '<S24>/tan(alpha)^2'
  real_T AccountNearInfiniteTangent1_a;
                                      // '<S24>/Account Near Infinite Tangent1'
  real_T NumZ_g;                       // '<S24>/Num Z'
  real_T XinBody;                      // '<S26>/X in Body'
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
  real_T rtb_WorldGain_b_c[6];
  real_T rtb_Body2World_k[6];
  int64_T b_value[6];
  int64_T b_value_c[6];
  int64_T varargin_1[6];
  char_T b_zeroDelimTopic[40];
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
  real_T WorldGain_i;                  // '<S40>/World Gain'
  real_T WorldGain;                    // '<S39>/World Gain'
  real_T WorldGain_km;                 // '<S38>/World Gain'
  real_T WorldGain_e;                  // '<S36>/World Gain'
  real_T IfForceSurfaceGains;          // '<S9>/If Force Surface Gains'
  real_T Divide8;                      // '<Root>/Divide8'
  real_T WorldGain_o;                  // '<S34>/World Gain'
  real_T WorldGain_p;                  // '<S33>/World Gain'
  real_T WorldGain_eo;                 // '<S29>/World Gain'
  real_T WorldGain_d;                  // '<S30>/World Gain'
  real_T WorldGain_b;                  // '<S28>/World Gain'
  real_T WorldGain_l;                  // '<S27>/World Gain'
  real_T WorldGain_h;                  // '<S26>/World Gain'
  real_T WorldGain_ky;                 // '<S25>/World Gain'
  real_T WorldGain_c;                  // '<S23>/World Gain'
  real_T WorldGain_ee;                 // '<S24>/World Gain'
  real_T Divide;                       // '<Root>/Divide'
  real_T q_idx_0;
  real_T q_idx_2;
  real_T q_idx_3;
  real_T q_idx_1;
  SL_Bus_std_msgs_Bool In1_p;          // '<S22>/In1'
  int64_T b_value_p;
  int64_T b_value_l;
  int32_T PulseGenerator;              // '<Root>/Pulse Generator'
  int32_T i;
  int32_T k;
  uint32_T len;
  ZCEventType zcEvent;
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
  int32_T clockTickCounter;            // '<Root>/Pulse Generator'
};

// Zero-crossing (trigger) state
struct PrevZCX_PID_T {
  ZCSigState PublishControl_Trig_ZCE;  // '<Root>/Publish Control'
  ZCSigState ControlSystem_Trig_ZCE;   // '<Root>/Control System'
  ZCSigState CheckifGainsHaveChanged_Trig_ZC;// '<Root>/Check if Gains Have Changed' 
};

// Invariant block signals for system '<Root>/Tranfsform Gains1'
struct ConstB_TranfsformGains1_PID_T {
  real_T Zero;                         // '<S23>/Zero'
  real_T Zero_g;                       // '<S24>/Zero'
  real_T Zero_gl;                      // '<S25>/Zero'
  real_T Zero_m;                       // '<S26>/Zero'
  real_T Zero_d;                       // '<S27>/Zero'
  real_T Zero_f;                       // '<S28>/Zero'
};

// Invariant block signals (default storage)
struct ConstB_PID_T {
  ConstB_TranfsformGains1_PID_T TranfsformGains3;// '<Root>/Tranfsform Gains3'
  ConstB_TranfsformGains1_PID_T TranfsformGains2;// '<Root>/Tranfsform Gains2'
  ConstB_TranfsformGains1_PID_T TranfsformGains1;// '<Root>/Tranfsform Gains1'
};

// Constant parameters (default storage)
struct ConstP_PID_T {
  // Computed Parameter: IdentityMatrix_IDMatrixData
  //  Referenced by: '<Root>/IdentityMatrix'

  real_T IdentityMatrix_IDMatrixData[9];
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
      uint16_T TID[2];
    } TaskCounters;
  } Timing;
};

extern const ConstB_PID_T PID_ConstB;  // constant block i/o

// Constant parameters (default storage)
extern const ConstP_PID_T PID_ConstP;

// Class declaration for model PID
class PID final
{
  // public data and function members
 public:
  // Copy Constructor
  PID(PID const&) = delete;

  // Assignment Operator
  PID& operator= (PID const&) & = delete;

  // Move Constructor
  PID(PID &&) = delete;

  // Move Assignment Operator
  PID& operator= (PID &&) = delete;

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

  // Triggered events
  PrevZCX_PID_T PID_PrevZCX;

  // private member function(s) for subsystem '<Root>/Tranfsform Gains1'
  void PID_TranfsformGains1(const real_T rtu_TwistBody[6], const real_T
    rtu_LinearRotm[9], const real_T rtu_AngularRotm[9], real_T
    *rty_WorldFrameGains, real_T *rty_WorldFrameGains_i, real_T
    *rty_WorldFrameGains_f, real_T *rty_WorldFrameGains_o, real_T
    *rty_WorldFrameGains_on, real_T *rty_WorldFrameGains_c,
    B_TranfsformGains1_PID_T *localB);

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
//  '<S23>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y1'
//  '<S24>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y2'
//  '<S25>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y3'
//  '<S26>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y4'
//  '<S27>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y5'
//  '<S28>'  : 'PID/Tranfsform Gains1/Get World Frame Gain Y6'
//  '<S29>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y1'
//  '<S30>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y2'
//  '<S31>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y3'
//  '<S32>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y4'
//  '<S33>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y5'
//  '<S34>'  : 'PID/Tranfsform Gains2/Get World Frame Gain Y6'
//  '<S35>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y1'
//  '<S36>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y2'
//  '<S37>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y3'
//  '<S38>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y4'
//  '<S39>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y5'
//  '<S40>'  : 'PID/Tranfsform Gains3/Get World Frame Gain Y6'

#endif                                 // RTW_HEADER_PID_h_

//
// File trailer for generated code.
//
// [EOF]
//
