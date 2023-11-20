//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SMC.cpp
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
#include "SMC.h"
#include "SMC_types.h"
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include <stddef.h>
#include "zero_crossing_types.h"
#include "SMC_private.h"

void SMC::SMC_SystemCore_setup_i5(ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[29] = { '/', 't', 'a', 'l', 'o', 's', '/', 'c', 'o',
    'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '/', 'g', 'e', 'n', 'e', 'r', 'a',
    't', 'e', 'd', 'R', '2' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 29; i++) {
    SMC_B.b_zeroDelimTopic_p[i] = tmp[i];
  }

  SMC_B.b_zeroDelimTopic_p[29] = '\x00';
  Pub_SMC_252.createPublisher(&SMC_B.b_zeroDelimTopic_p[0], qos_profile);
  obj->isSetupComplete = true;
}

void SMC::SMC_SystemCore_setup_i5v(ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[27] = { '/', 't', 'a', 'l', 'o', 's', '/', 'c', 'o',
    'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '/', 'c', 'u', 'r', 'v', 'e', 'T',
    'i', 'm', 'e' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 27; i++) {
    SMC_B.b_zeroDelimTopic_c[i] = tmp[i];
  }

  SMC_B.b_zeroDelimTopic_c[27] = '\x00';
  Pub_SMC_257.createPublisher(&SMC_B.b_zeroDelimTopic_c[0], qos_profile);
  obj->isSetupComplete = true;
}

void SMC::SMC_SystemCore_setup(ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[24] = { '/', 't', 'a', 'l', 'o', 's', '/', 'o', 'd',
    'o', 'm', 'e', 't', 'r', 'y', '/', 'f', 'i', 'l', 't', 'e', 'r', 'e', 'd' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 24; i++) {
    SMC_B.b_zeroDelimTopic_g[i] = tmp[i];
  }

  SMC_B.b_zeroDelimTopic_g[24] = '\x00';
  Sub_SMC_175.createSubscriber(&SMC_B.b_zeroDelimTopic_g[0], qos_profile);
  obj->isSetupComplete = true;
}

void SMC::SMC_SystemCore_setup_i5v0(ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[26] = { '/', 't', 'a', 'l', 'o', 's', '/', 'c', 'o',
    'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '/', 's', 'e', 't', 'p', 'o', 'i',
    'n', 't' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 26; i++) {
    SMC_B.b_zeroDelimTopic_f[i] = tmp[i];
  }

  SMC_B.b_zeroDelimTopic_f[26] = '\x00';
  Sub_SMC_171.createSubscriber(&SMC_B.b_zeroDelimTopic_f[0], qos_profile);
  obj->isSetupComplete = true;
}

void SMC::SMC_SystemCore_setup_i(ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[40] = { '/', 't', 'a', 'l', 'o', 's', '/', 'c', 'o',
    'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '/', 'a', 'c', 't', 'i', 'v', 'e',
    'C', 'o', 'n', 't', 'r', 'o', 'l', 'B', 'o', 'd', 'y', 'F', 'o', 'r', 'c',
    'e' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 40; i++) {
    SMC_B.b_zeroDelimTopic[i] = tmp[i];
  }

  SMC_B.b_zeroDelimTopic[40] = '\x00';
  Pub_SMC_179.createPublisher(&SMC_B.b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

// Model step function
void SMC::step()
{
  SL_Bus_std_msgs_Float32 rtb_BusAssignment2;
  real_T rtb_world2Body_tmp;
  real_T rtb_world2Body_tmp_0;
  real_T rtb_world2Body_tmp_1;
  real_T rtb_world2Body_tmp_2;
  int32_T i;
  uint32_T len;
  boolean_T b_p;
  boolean_T b_varargout_1;
  boolean_T exitg1;

  // MATLABSystem: '<Root>/mass3'
  ParamGet_SMC_197.getParameter(18U, &SMC_B.value[0], &len);

  // MATLABSystem: '<Root>/Scaling Parameter'
  ParamGet_SMC_208.getParameter(&SMC_B.value_g);

  // MATLABSystem: '<S2>/SourceBlock'
  b_varargout_1 = Sub_SMC_175.getLatestMessage(&SMC_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S2>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S7>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S7>/In1'
    SMC_B.In1 = SMC_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S2>/SourceBlock'
  // End of Outputs for SubSystem: '<S2>/Enabled Subsystem'

  // MATLABSystem: '<S6>/SourceBlock'
  b_varargout_1 = Sub_SMC_171.getLatestMessage(&SMC_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S25>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S25>/In1'
    SMC_B.In1_h = SMC_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'

  // MATLABSystem: '<Root>/vMax'
  ParamGet_SMC_240.getParameter(&SMC_B.value_l);

  // MATLABSystem: '<Root>/aMax'
  ParamGet_SMC_242.getParameter(&SMC_B.value_l);

  // MATLABSystem: '<Root>/jMax'
  ParamGet_SMC_244.getParameter(&SMC_B.value_l);

  // Clock: '<S5>/Clock'
  SMC_B.curveTimeOffset = (&SMC_M)->Timing.t[0];

  // MATLABSystem: '<Root>/mass1'
  ParamGet_SMC_192.getParameter(&SMC_B.value_l);

  // MATLABSystem: '<Root>/mass2'
  ParamGet_SMC_193.getParameter(&SMC_B.value_d);

  // SignalConversion generated from: '<S12>/ SFunction ' incorporates:
  //   MATLAB Function: '<S5>/MATLAB Function'

  SMC_B.setPoint[0] = SMC_B.In1_h.position.x;
  SMC_B.setPoint[1] = SMC_B.In1_h.position.y;
  SMC_B.setPoint[2] = SMC_B.In1_h.position.z;
  SMC_B.setPoint[3] = SMC_B.In1_h.orientation.x;
  SMC_B.setPoint[4] = SMC_B.In1_h.orientation.y;
  SMC_B.setPoint[5] = SMC_B.In1_h.orientation.z;
  SMC_B.setPoint[6] = SMC_B.In1_h.orientation.w;

  // MATLAB Function: '<S5>/MATLAB Function' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion1'
  //   DataTypeConversion: '<Root>/Data Type Conversion8'
  //   MATLABSystem: '<Root>/Scaling Parameter'
  //   MATLABSystem: '<Root>/mass1'
  //   Memory: '<S5>/Previous Curve Error'
  //   Memory: '<S5>/previousCurveTimeOffset'
  //   Memory: '<S5>/previousSetPoint'
  //   Product: '<Root>/Divide1'

  b_varargout_1 = false;
  b_p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 7)) {
    if (!(SMC_DW.previousSetPoint_PreviousInput[i] == SMC_B.setPoint[i])) {
      b_p = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (b_p) {
    b_varargout_1 = true;
  }

  if (!b_varargout_1) {
    SMC_B.scale = 0.0;
    SMC_B.resetCurve = true;
  } else {
    SMC_B.scale = 3.3121686421112381E-170;
    SMC_B.absxk = fabs(SMC_DW.PreviousCurveError_PreviousInpu[0]);
    if (SMC_B.absxk > 3.3121686421112381E-170) {
      SMC_B.q_idx_2 = 1.0;
      SMC_B.scale = SMC_B.absxk;
    } else {
      SMC_B.t = SMC_B.absxk / 3.3121686421112381E-170;
      SMC_B.q_idx_2 = SMC_B.t * SMC_B.t;
    }

    SMC_B.absxk = fabs(SMC_DW.PreviousCurveError_PreviousInpu[1]);
    if (SMC_B.absxk > SMC_B.scale) {
      SMC_B.t = SMC_B.scale / SMC_B.absxk;
      SMC_B.q_idx_2 = SMC_B.q_idx_2 * SMC_B.t * SMC_B.t + 1.0;
      SMC_B.scale = SMC_B.absxk;
    } else {
      SMC_B.t = SMC_B.absxk / SMC_B.scale;
      SMC_B.q_idx_2 += SMC_B.t * SMC_B.t;
    }

    SMC_B.absxk = fabs(SMC_DW.PreviousCurveError_PreviousInpu[2]);
    if (SMC_B.absxk > SMC_B.scale) {
      SMC_B.t = SMC_B.scale / SMC_B.absxk;
      SMC_B.q_idx_2 = SMC_B.q_idx_2 * SMC_B.t * SMC_B.t + 1.0;
      SMC_B.scale = SMC_B.absxk;
    } else {
      SMC_B.t = SMC_B.absxk / SMC_B.scale;
      SMC_B.q_idx_2 += SMC_B.t * SMC_B.t;
    }

    SMC_B.q_idx_2 = SMC_B.scale * sqrt(SMC_B.q_idx_2);
    if ((SMC_B.q_idx_2 > static_cast<real_T>(SMC_B.value_l) / static_cast<real_T>
         (SMC_B.value_g)) || (static_cast<real_T>(SMC_B.value_d) /
         static_cast<real_T>(SMC_B.value_g) < 8.7749643873921244)) {
      SMC_B.scale = 0.0;
      SMC_B.resetCurve = true;
    } else {
      for (i = 0; i < 7; i++) {
        SMC_B.setPoint[i] = SMC_DW.previousSetPoint_PreviousInput[i];
      }

      SMC_B.scale = SMC_B.curveTimeOffset -
        SMC_DW.previousCurveTimeOffset_Previou;

      // Clock: '<S5>/Clock' incorporates:
      //   Memory: '<S5>/previousCurveTimeOffset'
      //   Memory: '<S5>/previousSetPoint'

      SMC_B.curveTimeOffset = SMC_DW.previousCurveTimeOffset_Previou;
      SMC_B.resetCurve = false;
    }
  }

  // Outputs for Triggered SubSystem: '<S5>/Subsystem' incorporates:
  //   TriggerPort: '<S14>/Trigger'

  if (SMC_B.resetCurve && (SMC_PrevZCX.Subsystem_Trig_ZCE != POS_ZCSIG)) {
    // MATLAB Function: '<S14>/Generate Motion Profiles'
    for (i = 0; i < 6; i++) {
      SMC_B.R2[i] = 0.0;
    }

    SMC_B.tEst = 0.0;

    // End of MATLAB Function: '<S14>/Generate Motion Profiles'
  }

  SMC_PrevZCX.Subsystem_Trig_ZCE = SMC_B.resetCurve;

  // End of Outputs for SubSystem: '<S5>/Subsystem'

  // MATLAB Function: '<S13>/Calculate World 2 Body Rotm' incorporates:
  //   SignalConversion generated from: '<S15>/ SFunction '

  SMC_B.q_idx_3 = 1.0 / sqrt(((SMC_B.In1.pose.pose.orientation.w *
    SMC_B.In1.pose.pose.orientation.w + SMC_B.In1.pose.pose.orientation.x *
    SMC_B.In1.pose.pose.orientation.x) + SMC_B.In1.pose.pose.orientation.y *
    SMC_B.In1.pose.pose.orientation.y) + SMC_B.In1.pose.pose.orientation.z *
    SMC_B.In1.pose.pose.orientation.z);
  SMC_B.absxk = SMC_B.In1.pose.pose.orientation.w * SMC_B.q_idx_3;
  SMC_B.t = SMC_B.In1.pose.pose.orientation.x * SMC_B.q_idx_3;
  SMC_B.q_idx_2 = SMC_B.In1.pose.pose.orientation.y * SMC_B.q_idx_3;
  SMC_B.q_idx_3 *= SMC_B.In1.pose.pose.orientation.z;
  rtb_world2Body_tmp = SMC_B.q_idx_3 * SMC_B.q_idx_3;
  rtb_world2Body_tmp_2 = SMC_B.q_idx_2 * SMC_B.q_idx_2;
  SMC_B.world2Body[0] = 1.0 - (rtb_world2Body_tmp_2 + rtb_world2Body_tmp) * 2.0;
  SMC_B.rtb_world2Body_tmp = SMC_B.t * SMC_B.q_idx_2;
  SMC_B.rtb_world2Body_tmp_d = SMC_B.absxk * SMC_B.q_idx_3;
  SMC_B.world2Body[1] = (SMC_B.rtb_world2Body_tmp - SMC_B.rtb_world2Body_tmp_d) *
    2.0;
  rtb_world2Body_tmp_0 = SMC_B.t * SMC_B.q_idx_3;
  rtb_world2Body_tmp_1 = SMC_B.absxk * SMC_B.q_idx_2;
  SMC_B.world2Body[2] = (rtb_world2Body_tmp_0 + rtb_world2Body_tmp_1) * 2.0;
  SMC_B.world2Body[3] = (SMC_B.rtb_world2Body_tmp + SMC_B.rtb_world2Body_tmp_d) *
    2.0;
  SMC_B.rtb_world2Body_tmp = SMC_B.t * SMC_B.t;
  SMC_B.world2Body[4] = 1.0 - (SMC_B.rtb_world2Body_tmp + rtb_world2Body_tmp) *
    2.0;
  rtb_world2Body_tmp = SMC_B.q_idx_2 * SMC_B.q_idx_3;
  SMC_B.rtb_world2Body_tmp_d = SMC_B.absxk * SMC_B.t;
  SMC_B.world2Body[5] = (rtb_world2Body_tmp - SMC_B.rtb_world2Body_tmp_d) * 2.0;
  SMC_B.world2Body[6] = (rtb_world2Body_tmp_0 - rtb_world2Body_tmp_1) * 2.0;
  SMC_B.world2Body[7] = (rtb_world2Body_tmp + SMC_B.rtb_world2Body_tmp_d) * 2.0;
  SMC_B.world2Body[8] = 1.0 - (SMC_B.rtb_world2Body_tmp + rtb_world2Body_tmp_2) *
    2.0;
  memcpy(&SMC_B.catArgs[0], &SMC_B.world2Body[0], 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    int32_T rtb_world2Body_tmp_3;
    rtb_world2Body_tmp_3 = (static_cast<int8_T>(i + 1) - 1) * 3;
    SMC_B.world2Body[static_cast<int8_T>(i + 1) - 1] =
      SMC_B.catArgs[rtb_world2Body_tmp_3];
    SMC_B.world2Body[static_cast<int8_T>(i + 1) + 2] =
      SMC_B.catArgs[rtb_world2Body_tmp_3 + 1];
    SMC_B.world2Body[static_cast<int8_T>(i + 1) + 5] =
      SMC_B.catArgs[rtb_world2Body_tmp_3 + 2];
  }

  // End of MATLAB Function: '<S13>/Calculate World 2 Body Rotm'
  for (i = 0; i < 3; i++) {
    // Math: '<S13>/Body2World' incorporates:
    //   Math: '<S20>/Body2World Rotm'

    SMC_B.catArgs[3 * i] = SMC_B.world2Body[i];
    SMC_B.catArgs[3 * i + 1] = SMC_B.world2Body[i + 3];
    SMC_B.catArgs[3 * i + 2] = SMC_B.world2Body[i + 6];
  }

  // MATLAB Function: '<S13>/MATLAB Function'
  for (i = 0; i < 6; i++) {
    SMC_B.EError[i] = 0.0;
  }

  // MATLABSystem: '<Root>/mass5'
  ParamGet_SMC_202.getParameter(6U, &SMC_B.value_c[0], &len);
  for (i = 0; i < 6; i++) {
    // Product: '<Root>/Divide5' incorporates:
    //   DataTypeConversion: '<Root>/Data Type Conversion5'
    //   DataTypeConversion: '<Root>/Data Type Conversion8'
    //   MATLABSystem: '<Root>/Scaling Parameter'
    //   MATLABSystem: '<Root>/mass5'

    SMC_B.Divide5[i] = static_cast<real_T>(SMC_B.value_c[i]) /
      static_cast<real_T>(SMC_B.value_g);

    // Switch: '<S19>/Reset S Flip Time' incorporates:
    //   Constant: '<S19>/Constant'
    //   Memory: '<S19>/Previous S-Flip Time'

    if (SMC_B.resetCurve) {
      SMC_B.SFlipTime[i] = SMC_P.Constant_Value_i[i];
    } else {
      SMC_B.SFlipTime[i] = SMC_DW.PreviousSFlipTime_PreviousInput[i];
    }

    // End of Switch: '<S19>/Reset S Flip Time'
  }

  for (i = 0; i < 3; i++) {
    // Product: '<S19>/World Frame Linear Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i];
    SMC_B.t = SMC_B.absxk * SMC_B.Divide5[0];

    // Product: '<S19>/World Frame Angular Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.q_idx_2 = SMC_B.absxk * SMC_B.Divide5[3];

    // Product: '<S19>/World Frame Linear Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i + 3];
    SMC_B.t += SMC_B.absxk * SMC_B.Divide5[1];

    // Product: '<S19>/World Frame Angular Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.q_idx_2 += SMC_B.absxk * SMC_B.Divide5[4];

    // Product: '<S19>/World Frame Linear Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i + 6];

    // MATLAB Function: '<S19>/MATLAB Function' incorporates:
    //   MATLAB Function: '<S13>/MATLAB Function'
    //   Math: '<S13>/Body2World'
    //   Product: '<S19>/World Frame Angular Lamda'
    //   Product: '<S19>/World Frame Linear Lamda'
    //   SignalConversion generated from: '<S22>/ SFunction '

    SMC_B.SSign[i] = SMC_B.EError[i] - (SMC_B.absxk * SMC_B.Divide5[2] + SMC_B.t)
      * 0.0;
    SMC_B.SSign[i + 3] = SMC_B.EError[i + 3] - (SMC_B.absxk * SMC_B.Divide5[5] +
      SMC_B.q_idx_2) * 0.0;
  }

  // MATLAB Function: '<S19>/MATLAB Function' incorporates:
  //   Memory: '<S19>/Previous SSign'

  for (i = 0; i < 6; i++) {
    if (SMC_B.SSign[i] == 0.0) {
      SMC_B.SSign[i] = 1.0E-7;
    }

    SMC_B.SSign[i] /= fabs(SMC_B.SSign[i]);
    if (SMC_B.SSign[i] * SMC_DW.PreviousSSign_PreviousInput[i] < 0.0) {
      SMC_B.SFlipTime[i] = SMC_B.scale;
    }
  }

  // MATLABSystem: '<Root>/Eta 1'
  ParamGet_SMC_206.getParameter(6U, &SMC_B.value_c[0], &len);

  // Product: '<Root>/Divide7' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion7'
  //   DataTypeConversion: '<Root>/Data Type Conversion8'
  //   MATLABSystem: '<Root>/Eta 1'
  //   MATLABSystem: '<Root>/Scaling Parameter'
  //   SignalConversion generated from: '<S21>/ SFunction '

  for (i = 0; i < 6; i++) {
    SMC_B.TmpSignalConversionAtSFu_io[i] = static_cast<real_T>(SMC_B.value_c[i])
      / static_cast<real_T>(SMC_B.value_g);
  }

  // End of Product: '<Root>/Divide7'
  for (i = 0; i < 3; i++) {
    // Product: '<S19>/World Frame Linear Eta 1' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i];
    SMC_B.t = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[0];

    // Product: '<S19>/World Frame Angular Eta 1' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.q_idx_2 = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[3];

    // Product: '<S19>/World Frame Linear Eta 1' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i + 3];
    SMC_B.t += SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[1];

    // Product: '<S19>/World Frame Angular Eta 1' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.q_idx_2 += SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[4];

    // Product: '<S19>/World Frame Linear Eta 1' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i + 6];

    // Product: '<S19>/Product1' incorporates:
    //   Math: '<S13>/Body2World'
    //   Product: '<S19>/World Frame Angular Eta 1'
    //   Product: '<S19>/World Frame Linear Eta 1'

    SMC_B.catArgs_c[i] = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[2] +
      SMC_B.t;
    SMC_B.catArgs_c[i + 3] = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[5]
      + SMC_B.q_idx_2;
  }

  // Product: '<S19>/Product1' incorporates:
  //   Product: '<S19>/1st Order Correction'
  //   Sum: '<S13>/World Frame Control Force'

  for (i = 0; i < 6; i++) {
    SMC_B.WorldFrameControlForce[i] = SMC_B.SFlipTime[i] * SMC_B.SSign[i] *
      SMC_B.catArgs_c[i];
  }

  // MATLABSystem: '<Root>/Eta 0'
  ParamGet_SMC_204.getParameter(6U, &SMC_B.value_c[0], &len);

  // Product: '<Root>/Divide6' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion6'
  //   DataTypeConversion: '<Root>/Data Type Conversion8'
  //   MATLABSystem: '<Root>/Eta 0'
  //   MATLABSystem: '<Root>/Scaling Parameter'
  //   SignalConversion generated from: '<S21>/ SFunction '

  for (i = 0; i < 6; i++) {
    SMC_B.TmpSignalConversionAtSFu_io[i] = static_cast<real_T>(SMC_B.value_c[i])
      / static_cast<real_T>(SMC_B.value_g);
  }

  // End of Product: '<Root>/Divide6'

  // MATLABSystem: '<Root>/mass'
  ParamGet_SMC_190.getParameter(&SMC_B.value_l);

  // Product: '<Root>/Divide3' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   DataTypeConversion: '<Root>/Data Type Conversion8'
  //   MATLABSystem: '<Root>/Scaling Parameter'
  //   MATLABSystem: '<Root>/mass'

  SMC_B.scale = static_cast<real_T>(SMC_B.value_l) / static_cast<real_T>
    (SMC_B.value_g);

  // MATLABSystem: '<Root>/mass4'
  ParamGet_SMC_199.getParameter(3U, &SMC_B.value_m[0], &len);

  // Product: '<S20>/World Frame Drag Torques' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion4'
  //   DataTypeConversion: '<Root>/Data Type Conversion8'
  //   MATLABSystem: '<Root>/Scaling Parameter'
  //   MATLABSystem: '<Root>/mass4'
  //   Product: '<Root>/Divide4'

  SMC_B.q_idx_3 = static_cast<real_T>(SMC_B.value_m[0]) / static_cast<real_T>
    (SMC_B.value_g);
  rtb_world2Body_tmp = static_cast<real_T>(SMC_B.value_m[1]) /
    static_cast<real_T>(SMC_B.value_g);
  rtb_world2Body_tmp_2 = static_cast<real_T>(SMC_B.value_m[2]) /
    static_cast<real_T>(SMC_B.value_g);
  for (i = 0; i < 3; i++) {
    real_T catArgs;

    // Sum: '<S13>/World Frame Control Force' incorporates:
    //   MATLAB Function: '<S20>/MATLAB Function'
    //   Product: '<S20>/Linear Forces'

    SMC_B.dv[i] = 0.0 / SMC_B.scale;

    // Product: '<S20>/Angular Torques' incorporates:
    //   Math: '<S20>/Body2World Rotm'
    //   Product: '<S20>/World Frame Drag Torques'

    SMC_B.absxk = SMC_B.catArgs[i];
    SMC_B.rtb_world2Body_tmp = SMC_B.absxk * SMC_B.q_idx_3;

    // Product: '<S17>/World Frame Drag Forces' incorporates:
    //   MATLAB Function: '<S17>/calculate body frame drag forces'
    //   Product: '<S17>/World Frame Drag Torques'

    SMC_B.q_idx_2 = SMC_B.absxk * 0.0;

    // Product: '<S19>/World Frame Linear Eta 0' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.rtb_world2Body_tmp_d = SMC_B.absxk *
      SMC_B.TmpSignalConversionAtSFu_io[0];

    // Product: '<S19>/World Frame Angular Eta 0' incorporates:
    //   Math: '<S13>/Body2World'

    rtb_world2Body_tmp_0 = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[3];

    // Product: '<S16>/World Frame Linear Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    rtb_world2Body_tmp_1 = SMC_B.absxk * SMC_B.Divide5[0];

    // Product: '<S16>/World Frame Angular Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    catArgs = SMC_B.absxk * SMC_B.Divide5[3];

    // Product: '<S20>/Angular Torques' incorporates:
    //   Math: '<S20>/Body2World Rotm'
    //   Product: '<S20>/World Frame Drag Torques'

    SMC_B.absxk = SMC_B.catArgs[i + 3];
    SMC_B.rtb_world2Body_tmp += SMC_B.absxk * rtb_world2Body_tmp;

    // Product: '<S17>/World Frame Drag Forces' incorporates:
    //   MATLAB Function: '<S17>/calculate body frame drag forces'

    SMC_B.t = SMC_B.absxk * 0.0 + SMC_B.q_idx_2;

    // Product: '<S17>/World Frame Drag Torques' incorporates:
    //   MATLAB Function: '<S17>/calculate body frame drag forces'

    SMC_B.q_idx_2 += SMC_B.absxk * 0.0;

    // Product: '<S19>/World Frame Linear Eta 0' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.rtb_world2Body_tmp_d += SMC_B.absxk *
      SMC_B.TmpSignalConversionAtSFu_io[1];

    // Product: '<S19>/World Frame Angular Eta 0' incorporates:
    //   Math: '<S13>/Body2World'

    rtb_world2Body_tmp_0 += SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[4];

    // Product: '<S16>/World Frame Linear Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    rtb_world2Body_tmp_1 += SMC_B.absxk * SMC_B.Divide5[1];

    // Product: '<S16>/World Frame Angular Lamda' incorporates:
    //   Math: '<S13>/Body2World'

    catArgs += SMC_B.absxk * SMC_B.Divide5[4];

    // Product: '<S20>/Angular Torques' incorporates:
    //   Math: '<S20>/Body2World Rotm'

    SMC_B.absxk = SMC_B.catArgs[i + 6];

    // Sum: '<S13>/World Frame Control Force' incorporates:
    //   MATLAB Function: '<S17>/calculate body frame drag forces'
    //   MATLAB Function: '<S20>/MATLAB Function'
    //   Product: '<S17>/World Frame Drag Forces'
    //   Product: '<S17>/World Frame Drag Torques'
    //   Product: '<S20>/Angular Torques'
    //   Product: '<S20>/World Frame Drag Torques'

    SMC_B.dv[i + 3] = 0.0 / (SMC_B.absxk * rtb_world2Body_tmp_2 +
      SMC_B.rtb_world2Body_tmp);
    SMC_B.catArgs_c[i] = SMC_B.absxk * 0.0 + SMC_B.t;
    SMC_B.catArgs_c[i + 3] = SMC_B.absxk * 0.0 + SMC_B.q_idx_2;

    // Product: '<S19>/Product' incorporates:
    //   Math: '<S13>/Body2World'
    //   Product: '<S19>/World Frame Angular Eta 0'
    //   Product: '<S19>/World Frame Linear Eta 0'

    SMC_B.catArgs_k[i] = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[2] +
      SMC_B.rtb_world2Body_tmp_d;
    SMC_B.catArgs_k[i + 3] = SMC_B.absxk * SMC_B.TmpSignalConversionAtSFu_io[5]
      + rtb_world2Body_tmp_0;

    // Product: '<S16>/Correction' incorporates:
    //   MATLAB Function: '<S13>/MATLAB Function'
    //   Math: '<S13>/Body2World'
    //   Product: '<S16>/World Frame Angular Lamda'
    //   Product: '<S16>/World Frame Linear Lamda'

    SMC_B.dv1[i] = 0.0 / (SMC_B.absxk * SMC_B.Divide5[2] + rtb_world2Body_tmp_1);
    SMC_B.dv1[i + 3] = 0.0 / (SMC_B.absxk * SMC_B.Divide5[5] + catArgs);
  }

  // Sum: '<S13>/World Frame Control Force' incorporates:
  //   Product: '<S19>/Product'
  //   Sum: '<S19>/Sum'

  for (i = 0; i < 6; i++) {
    SMC_B.Divide5[i] = ((SMC_B.dv[i] - SMC_B.catArgs_c[i]) - (SMC_B.catArgs_k[i]
      * SMC_B.SSign[i] + SMC_B.WorldFrameControlForce[i])) + SMC_B.dv1[i];
  }

  // Outputs for Triggered SubSystem: '<Root>/Publish Curve Data' incorporates:
  //   TriggerPort: '<S4>/Trigger'

  for (i = 0; i < 6; i++) {
    SMC_B.zcEvent[i] = rt_ZCFcn(RISING_ZERO_CROSSING,
      &SMC_PrevZCX.PublishCurveData_Trig_ZCE[i],
      (SMC_B.EError[i]));
  }

  b_varargout_1 = false;
  for (i = 0; i < 6; i++) {
    b_varargout_1 = (b_varargout_1 || (SMC_B.zcEvent[i] != NO_ZCEVENT));
  }

  if (b_varargout_1) {
    // BusAssignment: '<S4>/Bus Assignment1'
    SMC_B.BusAssignment1.linear.x = SMC_B.R2[0];
    SMC_B.BusAssignment1.angular.x = SMC_B.R2[1];
    SMC_B.BusAssignment1.angular.y = SMC_B.R2[2];
    SMC_B.BusAssignment1.angular.z = SMC_B.R2[3];
    SMC_B.BusAssignment1.linear.y = SMC_B.R2[4];
    SMC_B.BusAssignment1.linear.z = SMC_B.R2[5];

    // MATLABSystem: '<S9>/SinkBlock'
    Pub_SMC_252.publish(&SMC_B.BusAssignment1);

    // BusAssignment: '<S4>/Bus Assignment2' incorporates:
    //   DataTypeConversion: '<S4>/Data Type Conversion1'

    rtb_BusAssignment2.data = static_cast<real32_T>(SMC_B.tEst);

    // MATLABSystem: '<S10>/SinkBlock'
    Pub_SMC_257.publish(&rtb_BusAssignment2);
  }

  // End of Outputs for SubSystem: '<Root>/Publish Curve Data'
  for (i = 0; i < 3; i++) {
    // Product: '<S13>/Linear World To Body' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i];
    SMC_B.scale = SMC_B.absxk * SMC_B.Divide5[0];

    // Product: '<S13>/Angular World To Body' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.t = SMC_B.absxk * SMC_B.Divide5[3];

    // Product: '<S13>/Linear World To Body' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i + 3];
    SMC_B.scale += SMC_B.absxk * SMC_B.Divide5[1];

    // Product: '<S13>/Angular World To Body' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.t += SMC_B.absxk * SMC_B.Divide5[4];

    // Product: '<S13>/Linear World To Body' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.absxk = SMC_B.catArgs[i + 6];
    SMC_B.LinearWorldToBody[i] = SMC_B.absxk * SMC_B.Divide5[2] + SMC_B.scale;

    // Product: '<S13>/Angular World To Body' incorporates:
    //   Math: '<S13>/Body2World'

    SMC_B.AngularWorldToBody[i] = SMC_B.absxk * SMC_B.Divide5[5] + SMC_B.t;
  }

  // BusAssignment: '<Root>/Bus Assignment'
  SMC_B.BusAssignment.linear.x = SMC_B.LinearWorldToBody[0];
  SMC_B.BusAssignment.linear.y = SMC_B.LinearWorldToBody[1];
  SMC_B.BusAssignment.linear.z = SMC_B.LinearWorldToBody[2];
  SMC_B.BusAssignment.angular.x = SMC_B.AngularWorldToBody[0];
  SMC_B.BusAssignment.angular.y = SMC_B.AngularWorldToBody[1];
  SMC_B.BusAssignment.angular.z = SMC_B.AngularWorldToBody[2];

  // MATLABSystem: '<S3>/SinkBlock'
  Pub_SMC_179.publish(&SMC_B.BusAssignment);

  // Update for Memory: '<S5>/Previous Curve Error'
  for (int32_T i = 0; i < 6; i++) {
    SMC_DW.PreviousCurveError_PreviousInpu[i] = SMC_B.EError[i];
  }

  // End of Update for Memory: '<S5>/Previous Curve Error'

  // Update for Memory: '<S5>/previousCurveTimeOffset'
  SMC_DW.previousCurveTimeOffset_Previou = SMC_B.curveTimeOffset;

  // Update for Memory: '<S5>/previousSetPoint'
  for (int32_T i = 0; i < 7; i++) {
    SMC_DW.previousSetPoint_PreviousInput[i] = SMC_B.setPoint[i];
  }

  // End of Update for Memory: '<S5>/previousSetPoint'
  for (int32_T i = 0; i < 6; i++) {
    // Update for Memory: '<S19>/Previous S-Flip Time'
    SMC_DW.PreviousSFlipTime_PreviousInput[i] = SMC_B.SFlipTime[i];

    // Update for Memory: '<S19>/Previous SSign'
    SMC_DW.PreviousSSign_PreviousInput[i] = SMC_B.SSign[i];
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  (&SMC_M)->Timing.t[0] =
    ((time_T)(++(&SMC_M)->Timing.clockTick0)) * (&SMC_M)->Timing.stepSize0;

  {
    // Update absolute timer for sample time: [0.2s, 0.0s]
    // The "clockTick1" counts the number of times the code of this task has
    //  been executed. The resolution of this integer timer is 0.2, which is the step size
    //  of the task. Size of "clockTick1" ensures timer will not overflow during the
    //  application lifespan selected.

    (&SMC_M)->Timing.clockTick1++;
  }
}

// Model initialize function
void SMC::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&SMC_M)->solverInfo, &(&SMC_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&SMC_M)->solverInfo, &rtmGetTPtr((&SMC_M)));
    rtsiSetStepSizePtr(&(&SMC_M)->solverInfo, &(&SMC_M)->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&(&SMC_M)->solverInfo, (&rtmGetErrorStatus((&SMC_M))));
    rtsiSetRTModelPtr(&(&SMC_M)->solverInfo, (&SMC_M));
  }

  rtsiSetSimTimeStep(&(&SMC_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&(&SMC_M)->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr((&SMC_M), &(&SMC_M)->Timing.tArray[0]);
  (&SMC_M)->Timing.stepSize0 = 0.2;

  {
    int64_T varargin_1;
    char_T prmName_0[6];
    char_T prmName[5];
    static const char_T tmp[16] = { 'd', 'r', 'a', 'g', '_', 'c', 'o', 'e', 'f',
      'f', 'i', 'c', 'e', 'n', 't', 's' };

    static const char_T tmp_0[24] = { 'p', 'a', 'r', 'a', 'm', 'e', 't', 'e',
      'r', '_', 's', 'c', 'a', 'l', 'i', 'n', 'g', '_', 'f', 'a', 'c', 't', 'o',
      'r' };

    static const char_T tmp_1[31] = { 'a', 'n', 'g', 'u', 'l', 'a', 'r', '_',
      'r', 'e', 'g', 'e', 'n', 'e', 'r', 'a', 't', 'i', 'o', 'n', '_', 't', 'h',
      'r', 'e', 's', 'h', 'h', 'o', 'l', 'd' };

    static const char_T tmp_2[30] = { 'l', 'i', 'n', 'e', 'a', 'r', '_', 'r',
      'e', 'g', 'e', 'n', 'e', 'r', 'a', 't', 'i', 'o', 'n', '_', 't', 'h', 'r',
      'e', 's', 'h', 'h', 'o', 'l', 'd' };

    static const char_T tmp_3[5] = { 'l', 'a', 'm', 'd', 'a' };

    static const char_T tmp_4[11] = { 'E', 't', 'a', '_', 'O', 'r', 'd', 'e',
      'r', '_', '1' };

    static const char_T tmp_5[11] = { 'E', 't', 'a', '_', 'O', 'r', 'd', 'e',
      'r', '_', '0' };

    static const char_T tmp_6[10] = { 't', 'a', 'l', 'o', 's', '_', 'm', 'a',
      's', 's' };

    static const char_T tmp_7[19] = { 'r', 'o', 't', 'a', 't', 'i', 'o', 'n',
      'a', 'l', '_', 'i', 'n', 't', 'e', 'r', 'i', 'a', 's' };

    SMC_PrevZCX.Subsystem_Trig_ZCE = POS_ZCSIG;

    // InitializeConditions for Memory: '<S5>/Previous Curve Error'
    for (int32_T i = 0; i < 6; i++) {
      SMC_PrevZCX.PublishCurveData_Trig_ZCE[i] = UNINITIALIZED_ZCSIG;
      SMC_DW.PreviousCurveError_PreviousInpu[i] =
        SMC_P.PreviousCurveError_InitialCondi;
    }

    // End of InitializeConditions for Memory: '<S5>/Previous Curve Error'

    // InitializeConditions for Memory: '<S5>/previousCurveTimeOffset'
    SMC_DW.previousCurveTimeOffset_Previou =
      SMC_P.previousCurveTimeOffset_Initial;

    // InitializeConditions for Memory: '<S5>/previousSetPoint'
    for (int32_T i = 0; i < 7; i++) {
      SMC_DW.previousSetPoint_PreviousInput[i] =
        SMC_P.previousSetPoint_InitialConditi;
    }

    // End of InitializeConditions for Memory: '<S5>/previousSetPoint'

    // SystemInitialize for Enabled SubSystem: '<S2>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S7>/In1' incorporates:
    //   Outport: '<S7>/Out1'

    SMC_B.In1 = SMC_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S2>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S6>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S25>/In1' incorporates:
    //   Outport: '<S25>/Out1'

    SMC_B.In1_h = SMC_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S6>/Enabled Subsystem'

    // SystemInitialize for Triggered SubSystem: '<S5>/Subsystem'
    for (int32_T i = 0; i < 6; i++) {
      // InitializeConditions for Memory: '<S19>/Previous S-Flip Time'
      SMC_DW.PreviousSFlipTime_PreviousInput[i] =
        SMC_P.PreviousSFlipTime_InitialCondit[i];

      // InitializeConditions for Memory: '<S19>/Previous SSign'
      SMC_DW.PreviousSSign_PreviousInput[i] =
        SMC_P.PreviousSSign_InitialCondition[i];

      // SystemInitialize for Outport: '<S14>/R2'
      SMC_B.R2[i] = SMC_P.R2_Y0;
    }

    // SystemInitialize for Outport: '<S14>/tEst'
    SMC_B.tEst = SMC_P.tEst_Y0;

    // End of SystemInitialize for SubSystem: '<S5>/Subsystem'

    // SystemInitialize for Triggered SubSystem: '<Root>/Publish Curve Data'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    SMC_DW.obj_fq.isInitialized = 0;
    SMC_DW.obj_fq.matlabCodegenIsDeleted = false;
    SMC_SystemCore_setup_i5(&SMC_DW.obj_fq);

    // Start for MATLABSystem: '<S10>/SinkBlock'
    SMC_DW.obj_ec.isInitialized = 0;
    SMC_DW.obj_ec.matlabCodegenIsDeleted = false;
    SMC_SystemCore_setup_i5v(&SMC_DW.obj_ec);

    // End of SystemInitialize for SubSystem: '<Root>/Publish Curve Data'

    // Start for MATLABSystem: '<Root>/mass3'
    SMC_DW.obj_j.matlabCodegenIsDeleted = false;
    SMC_DW.obj_j.isInitialized = 1;
    for (int32_T i = 0; i < 16; i++) {
      SMC_B.prmName_p[i] = tmp[i];
    }

    SMC_B.prmName_p[16] = '\x00';
    varargin_1 = 0LL;
    ParamGet_SMC_197.initParam(&SMC_B.prmName_p[0]);
    ParamGet_SMC_197.setInitialValue(&varargin_1, 1U);
    SMC_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/mass3'

    // Start for MATLABSystem: '<Root>/Scaling Parameter'
    SMC_DW.obj_k.matlabCodegenIsDeleted = false;
    SMC_DW.obj_k.isInitialized = 1;
    for (int32_T i = 0; i < 24; i++) {
      SMC_B.prmName_g[i] = tmp_0[i];
    }

    SMC_B.prmName_g[24] = '\x00';
    ParamGet_SMC_208.initParam(&SMC_B.prmName_g[0]);
    ParamGet_SMC_208.setInitialValue(1000000LL);
    SMC_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Scaling Parameter'

    // Start for MATLABSystem: '<S2>/SourceBlock'
    SMC_DW.obj_n3.isInitialized = 0;
    SMC_DW.obj_n3.matlabCodegenIsDeleted = false;
    SMC_SystemCore_setup(&SMC_DW.obj_n3);

    // Start for MATLABSystem: '<S6>/SourceBlock'
    SMC_DW.obj_d2.isInitialized = 0;
    SMC_DW.obj_d2.matlabCodegenIsDeleted = false;
    SMC_SystemCore_setup_i5v0(&SMC_DW.obj_d2);

    // Start for MATLABSystem: '<Root>/vMax'
    SMC_DW.obj.matlabCodegenIsDeleted = false;
    SMC_DW.obj.isInitialized = 1;
    prmName[0] = 'v';
    prmName[1] = 'M';
    prmName[2] = 'a';
    prmName[3] = 'x';
    prmName[4] = '\x00';
    ParamGet_SMC_240.initParam(&prmName[0]);
    ParamGet_SMC_240.setInitialValue(0LL);
    SMC_DW.obj.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/aMax'
    SMC_DW.obj_g.matlabCodegenIsDeleted = false;
    SMC_DW.obj_g.isInitialized = 1;
    prmName[0] = 'a';
    prmName[1] = 'M';
    prmName[2] = 'a';
    prmName[3] = 'x';
    prmName[4] = '\x00';
    ParamGet_SMC_242.initParam(&prmName[0]);
    ParamGet_SMC_242.setInitialValue(0LL);
    SMC_DW.obj_g.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/jMax'
    SMC_DW.obj_n.matlabCodegenIsDeleted = false;
    SMC_DW.obj_n.isInitialized = 1;
    prmName[0] = 'j';
    prmName[1] = 'M';
    prmName[2] = 'a';
    prmName[3] = 'x';
    prmName[4] = '\x00';
    ParamGet_SMC_244.initParam(&prmName[0]);
    ParamGet_SMC_244.setInitialValue(0LL);
    SMC_DW.obj_n.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/mass1'
    SMC_DW.obj_m.matlabCodegenIsDeleted = false;
    SMC_DW.obj_m.isInitialized = 1;
    for (int32_T i = 0; i < 31; i++) {
      SMC_B.prmName[i] = tmp_1[i];
    }

    SMC_B.prmName[31] = '\x00';
    ParamGet_SMC_192.initParam(&SMC_B.prmName[0]);
    ParamGet_SMC_192.setInitialValue(0LL);
    SMC_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/mass1'

    // Start for MATLABSystem: '<Root>/mass2'
    SMC_DW.obj_jf.matlabCodegenIsDeleted = false;
    SMC_DW.obj_jf.isInitialized = 1;
    for (int32_T i = 0; i < 30; i++) {
      SMC_B.prmName_b[i] = tmp_2[i];
    }

    SMC_B.prmName_b[30] = '\x00';
    ParamGet_SMC_193.initParam(&SMC_B.prmName_b[0]);
    ParamGet_SMC_193.setInitialValue(0LL);
    SMC_DW.obj_jf.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/mass2'

    // Start for MATLABSystem: '<Root>/mass5'
    SMC_DW.obj_d.matlabCodegenIsDeleted = false;
    SMC_DW.obj_d.isInitialized = 1;
    for (int32_T i = 0; i < 5; i++) {
      prmName_0[i] = tmp_3[i];
    }

    prmName_0[5] = '\x00';
    varargin_1 = 0LL;
    ParamGet_SMC_202.initParam(&prmName_0[0]);
    ParamGet_SMC_202.setInitialValue(&varargin_1, 1U);
    SMC_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/mass5'

    // Start for MATLABSystem: '<Root>/Eta 1'
    SMC_DW.obj_i.matlabCodegenIsDeleted = false;
    SMC_DW.obj_i.isInitialized = 1;
    for (int32_T i = 0; i < 11; i++) {
      SMC_B.prmName_l[i] = tmp_4[i];
    }

    SMC_B.prmName_l[11] = '\x00';
    varargin_1 = 0LL;
    ParamGet_SMC_206.initParam(&SMC_B.prmName_l[0]);
    ParamGet_SMC_206.setInitialValue(&varargin_1, 1U);
    SMC_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Eta 1'

    // Start for MATLABSystem: '<Root>/Eta 0'
    SMC_DW.obj_mc.matlabCodegenIsDeleted = false;
    SMC_DW.obj_mc.isInitialized = 1;
    for (int32_T i = 0; i < 11; i++) {
      SMC_B.prmName_l[i] = tmp_5[i];
    }

    SMC_B.prmName_l[11] = '\x00';
    varargin_1 = 0LL;
    ParamGet_SMC_204.initParam(&SMC_B.prmName_l[0]);
    ParamGet_SMC_204.setInitialValue(&varargin_1, 1U);
    SMC_DW.obj_mc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Eta 0'

    // Start for MATLABSystem: '<Root>/mass'
    SMC_DW.obj_e.matlabCodegenIsDeleted = false;
    SMC_DW.obj_e.isInitialized = 1;
    for (int32_T i = 0; i < 10; i++) {
      SMC_B.prmName_j[i] = tmp_6[i];
    }

    SMC_B.prmName_j[10] = '\x00';
    ParamGet_SMC_190.initParam(&SMC_B.prmName_j[0]);
    ParamGet_SMC_190.setInitialValue(0LL);
    SMC_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/mass'

    // Start for MATLABSystem: '<Root>/mass4'
    SMC_DW.obj_f.matlabCodegenIsDeleted = false;
    SMC_DW.obj_f.isInitialized = 1;
    for (int32_T i = 0; i < 19; i++) {
      SMC_B.prmName_n[i] = tmp_7[i];
    }

    SMC_B.prmName_n[19] = '\x00';
    varargin_1 = 0LL;
    ParamGet_SMC_199.initParam(&SMC_B.prmName_n[0]);
    ParamGet_SMC_199.setInitialValue(&varargin_1, 1U);
    SMC_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/mass4'

    // Start for MATLABSystem: '<S3>/SinkBlock'
    SMC_DW.obj_dj.isInitialized = 0;
    SMC_DW.obj_dj.matlabCodegenIsDeleted = false;
    SMC_SystemCore_setup_i(&SMC_DW.obj_dj);
  }
}

// Model terminate function
void SMC::terminate()
{
  // Terminate for MATLABSystem: '<Root>/mass3'
  if (!SMC_DW.obj_j.matlabCodegenIsDeleted) {
    SMC_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/mass3'

  // Terminate for MATLABSystem: '<Root>/Scaling Parameter'
  if (!SMC_DW.obj_k.matlabCodegenIsDeleted) {
    SMC_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Scaling Parameter'

  // Terminate for MATLABSystem: '<S2>/SourceBlock'
  if (!SMC_DW.obj_n3.matlabCodegenIsDeleted) {
    SMC_DW.obj_n3.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S2>/SourceBlock'

  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  if (!SMC_DW.obj_d2.matlabCodegenIsDeleted) {
    SMC_DW.obj_d2.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S6>/SourceBlock'

  // Terminate for MATLABSystem: '<Root>/vMax'
  if (!SMC_DW.obj.matlabCodegenIsDeleted) {
    SMC_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/vMax'

  // Terminate for MATLABSystem: '<Root>/aMax'
  if (!SMC_DW.obj_g.matlabCodegenIsDeleted) {
    SMC_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/aMax'

  // Terminate for MATLABSystem: '<Root>/jMax'
  if (!SMC_DW.obj_n.matlabCodegenIsDeleted) {
    SMC_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/jMax'

  // Terminate for MATLABSystem: '<Root>/mass1'
  if (!SMC_DW.obj_m.matlabCodegenIsDeleted) {
    SMC_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/mass1'

  // Terminate for MATLABSystem: '<Root>/mass2'
  if (!SMC_DW.obj_jf.matlabCodegenIsDeleted) {
    SMC_DW.obj_jf.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/mass2'

  // Terminate for MATLABSystem: '<Root>/mass5'
  if (!SMC_DW.obj_d.matlabCodegenIsDeleted) {
    SMC_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/mass5'

  // Terminate for MATLABSystem: '<Root>/Eta 1'
  if (!SMC_DW.obj_i.matlabCodegenIsDeleted) {
    SMC_DW.obj_i.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Eta 1'

  // Terminate for MATLABSystem: '<Root>/Eta 0'
  if (!SMC_DW.obj_mc.matlabCodegenIsDeleted) {
    SMC_DW.obj_mc.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Eta 0'

  // Terminate for MATLABSystem: '<Root>/mass'
  if (!SMC_DW.obj_e.matlabCodegenIsDeleted) {
    SMC_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/mass'

  // Terminate for MATLABSystem: '<Root>/mass4'
  if (!SMC_DW.obj_f.matlabCodegenIsDeleted) {
    SMC_DW.obj_f.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/mass4'

  // Terminate for Triggered SubSystem: '<Root>/Publish Curve Data'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  if (!SMC_DW.obj_fq.matlabCodegenIsDeleted) {
    SMC_DW.obj_fq.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S9>/SinkBlock'

  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  if (!SMC_DW.obj_ec.matlabCodegenIsDeleted) {
    SMC_DW.obj_ec.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S10>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish Curve Data'

  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  if (!SMC_DW.obj_dj.matlabCodegenIsDeleted) {
    SMC_DW.obj_dj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/SinkBlock'
}

// Constructor
SMC::SMC() :
  SMC_B(),
  SMC_DW(),
  SMC_PrevZCX(),
  SMC_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
SMC::~SMC()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_SMC_T * SMC::getRTM()
{
  return (&SMC_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
