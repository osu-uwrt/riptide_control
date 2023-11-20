//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SMC.h
//
// Code generated for Simulink model 'SMC'.
//
// Model version                  : 1.7
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Sun Nov 19 17:55:18 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 10
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SMC_h_
#define RTW_HEADER_SMC_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "SMC_types.h"
#include "rt_zcfcn.h"
#include <stddef.h>
#include "zero_crossing_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

// Block signals (default storage)
struct B_SMC_T {
  SL_Bus_nav_msgs_Odometry In1;        // '<S7>/In1'
  SL_Bus_nav_msgs_Odometry b_varargout_2;
  int64_T value[18];
  real_T catArgs[9];
  real_T world2Body[9];                // '<S13>/Calculate World 2 Body Rotm'
  SL_Bus_geometry_msgs_Pose In1_h;     // '<S25>/In1'
  SL_Bus_geometry_msgs_Pose b_varargout_2_m;
  SL_Bus_geometry_msgs_Twist BusAssignment1;// '<S4>/Bus Assignment1'
  SL_Bus_geometry_msgs_Twist BusAssignment;// '<Root>/Bus Assignment'
  real_T R2[6];                        // '<S14>/Generate Motion Profiles'
  real_T SSign[6];                     // '<S19>/MATLAB Function'
  real_T EError[6];                    // '<S13>/MATLAB Function'
  real_T setPoint[7];                  // '<S5>/MATLAB Function'
  real_T TmpSignalConversionAtSFu_io[6];
                                    // '<S17>/calculate body frame drag forces'
  real_T Divide5[6];                   // '<Root>/Divide5'
  real_T WorldFrameControlForce[6];    // '<S13>/World Frame Control Force'
  real_T catArgs_c[6];
  real_T dv[6];
  real_T catArgs_k[6];
  real_T dv1[6];
  int64_T value_c[6];
  char_T b_zeroDelimTopic[41];
  char_T prmName[32];
  char_T prmName_b[31];
  char_T b_zeroDelimTopic_p[30];
  char_T b_zeroDelimTopic_c[28];
  char_T b_zeroDelimTopic_f[27];
  char_T prmName_g[25];
  char_T b_zeroDelimTopic_g[25];
  ZCEventType zcEvent[6];
  real_T AngularWorldToBody[3];        // '<S13>/Angular World To Body'
  real_T LinearWorldToBody[3];         // '<S13>/Linear World To Body'
  int64_T value_m[3];
  char_T prmName_n[20];
  char_T prmName_p[17];
  char_T prmName_l[12];
  char_T prmName_j[11];
  real_T tEst;                         // '<S14>/Generate Motion Profiles'
  real_T SFlipTime[6];                 // '<S19>/MATLAB Function'
  real_T curveTimeOffset;              // '<S5>/MATLAB Function'
  real_T scale;
  real_T absxk;
  real_T t;
  real_T q_idx_3;
  real_T q_idx_2;
  real_T rtb_world2Body_tmp;
  real_T rtb_world2Body_tmp_d;
  int64_T value_g;
  int64_T value_l;
  int64_T value_d;
  boolean_T resetCurve;                // '<S5>/MATLAB Function'
};

// Block states (default storage) for system '<Root>'
struct DW_SMC_T {
  ros_slros2_internal_block_Get_T obj; // '<Root>/vMax'
  ros_slros2_internal_block_Get_T obj_d;// '<Root>/mass5'
  ros_slros2_internal_block_Get_T obj_f;// '<Root>/mass4'
  ros_slros2_internal_block_Get_T obj_j;// '<Root>/mass3'
  ros_slros2_internal_block_Get_T obj_jf;// '<Root>/mass2'
  ros_slros2_internal_block_Get_T obj_m;// '<Root>/mass1'
  ros_slros2_internal_block_Get_T obj_e;// '<Root>/mass'
  ros_slros2_internal_block_Get_T obj_n;// '<Root>/jMax'
  ros_slros2_internal_block_Get_T obj_g;// '<Root>/aMax'
  ros_slros2_internal_block_Get_T obj_k;// '<Root>/Scaling Parameter'
  ros_slros2_internal_block_Get_T obj_i;// '<Root>/Eta 1'
  ros_slros2_internal_block_Get_T obj_mc;// '<Root>/Eta 0'
  ros_slros2_internal_block_Sub_T obj_d2;// '<S6>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_n3;// '<S2>/SourceBlock'
  ros_slros2_internal_block_Pub_T obj_ec;// '<S10>/SinkBlock'
  ros_slros2_internal_block_Pub_T obj_fq;// '<S9>/SinkBlock'
  ros_slros2_internal_block_Pub_T obj_dj;// '<S3>/SinkBlock'
  real_T PreviousCurveError_PreviousInpu[6];// '<S5>/Previous Curve Error'
  real_T previousCurveTimeOffset_Previou;// '<S5>/previousCurveTimeOffset'
  real_T previousSetPoint_PreviousInput[7];// '<S5>/previousSetPoint'
  real_T PreviousSFlipTime_PreviousInput[6];// '<S19>/Previous S-Flip Time'
  real_T PreviousSSign_PreviousInput[6];// '<S19>/Previous SSign'
};

// Zero-crossing (trigger) state
struct PrevZCX_SMC_T {
  ZCSigState Subsystem_Trig_ZCE;       // '<S5>/Subsystem'
  ZCSigState PublishCurveData_Trig_ZCE[6];// '<Root>/Publish Curve Data'
};

// Parameters (default storage)
struct P_SMC_T_ {
  SL_Bus_nav_msgs_Odometry Out1_Y0;    // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S7>/Out1'

  SL_Bus_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                             //  Referenced by: '<S2>/Constant'

  SL_Bus_geometry_msgs_Pose Out1_Y0_a; // Computed Parameter: Out1_Y0_a
                                          //  Referenced by: '<S25>/Out1'

  SL_Bus_geometry_msgs_Pose Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                //  Referenced by: '<S6>/Constant'

  SL_Bus_geometry_msgs_Twist Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                 //  Referenced by: '<S11>/Constant'

  SL_Bus_geometry_msgs_Twist Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                 //  Referenced by: '<S1>/Constant'

  SL_Bus_std_msgs_Float32 Constant_Value_g;// Computed Parameter: Constant_Value_g
                                              //  Referenced by: '<S8>/Constant'

  real_T motionPlanningCoefficents_Y0;
                             // Computed Parameter: motionPlanningCoefficents_Y0
                                //  Referenced by: '<S14>/motionPlanningCoefficents'

  real_T R2_Y0;                        // Computed Parameter: R2_Y0
                                          //  Referenced by: '<S14>/R2'

  real_T tEst_Y0;                      // Computed Parameter: tEst_Y0
                                          //  Referenced by: '<S14>/tEst'

  real_T PreviousCurveError_InitialCondi;// Expression: 0
                                            //  Referenced by: '<S5>/Previous Curve Error'

  real_T previousCurveTimeOffset_Initial;// Expression: 0
                                            //  Referenced by: '<S5>/previousCurveTimeOffset'

  real_T previousSetPoint_InitialConditi;// Expression: 0
                                            //  Referenced by: '<S5>/previousSetPoint'

  real_T Constant_Value_i[6];          // Expression: [0;0;0;0;0;0]
                                          //  Referenced by: '<S19>/Constant'

  real_T PreviousSFlipTime_InitialCondit[6];// Expression: [0;0;0;0;0;0]
                                               //  Referenced by: '<S19>/Previous S-Flip Time'

  real_T PreviousSSign_InitialCondition[6];// Expression: [1;1;1;1;1;1]
                                              //  Referenced by: '<S19>/Previous SSign'

};

// Real-time Model Data Structure
struct tag_RTM_SMC_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Class declaration for model SMC
class SMC
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_SMC_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SMC();

  // Destructor
  ~SMC();

  // private data and function members
 private:
  // Block signals
  B_SMC_T SMC_B;

  // Block states
  DW_SMC_T SMC_DW;

  // Tunable parameters
  static P_SMC_T SMC_P;

  // Triggered events
  PrevZCX_SMC_T SMC_PrevZCX;

  // private member function(s) for subsystem '<Root>'
  void SMC_SystemCore_setup_i5(ros_slros2_internal_block_Pub_T *obj);
  void SMC_SystemCore_setup_i5v(ros_slros2_internal_block_Pub_T *obj);
  void SMC_SystemCore_setup(ros_slros2_internal_block_Sub_T *obj);
  void SMC_SystemCore_setup_i5v0(ros_slros2_internal_block_Sub_T *obj);
  void SMC_SystemCore_setup_i(ros_slros2_internal_block_Pub_T *obj);

  // Real-Time Model
  RT_MODEL_SMC_T SMC_M;
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
//  '<Root>' : 'SMC'
//  '<S1>'   : 'SMC/Blank Message'
//  '<S2>'   : 'SMC/Odometry Sub'
//  '<S3>'   : 'SMC/Publish Active Control Forces'
//  '<S4>'   : 'SMC/Publish Curve Data'
//  '<S5>'   : 'SMC/SMC'
//  '<S6>'   : 'SMC/Set Point Sub'
//  '<S7>'   : 'SMC/Odometry Sub/Enabled Subsystem'
//  '<S8>'   : 'SMC/Publish Curve Data/Blank Message1'
//  '<S9>'   : 'SMC/Publish Curve Data/Publish'
//  '<S10>'  : 'SMC/Publish Curve Data/Publish1'
//  '<S11>'  : 'SMC/Publish Curve Data/R2 Message'
//  '<S12>'  : 'SMC/SMC/MATLAB Function'
//  '<S13>'  : 'SMC/SMC/Mathmatical Model'
//  '<S14>'  : 'SMC/SMC/Subsystem'
//  '<S15>'  : 'SMC/SMC/Mathmatical Model/Calculate World 2 Body Rotm'
//  '<S16>'  : 'SMC/SMC/Mathmatical Model/Edot'
//  '<S17>'  : 'SMC/SMC/Mathmatical Model/Fm'
//  '<S18>'  : 'SMC/SMC/Mathmatical Model/MATLAB Function'
//  '<S19>'  : 'SMC/SMC/Mathmatical Model/S Block'
//  '<S20>'  : 'SMC/SMC/Mathmatical Model/Xd'
//  '<S21>'  : 'SMC/SMC/Mathmatical Model/Fm/calculate body frame drag forces'
//  '<S22>'  : 'SMC/SMC/Mathmatical Model/S Block/MATLAB Function'
//  '<S23>'  : 'SMC/SMC/Mathmatical Model/Xd/MATLAB Function'
//  '<S24>'  : 'SMC/SMC/Subsystem/Generate Motion Profiles'
//  '<S25>'  : 'SMC/Set Point Sub/Enabled Subsystem'

#endif                                 // RTW_HEADER_SMC_h_

//
// File trailer for generated code.
//
// [EOF]
//
