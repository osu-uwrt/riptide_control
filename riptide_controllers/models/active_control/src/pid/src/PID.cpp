//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID.cpp
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
#include "PID.h"
#include "rtwtypes.h"
#include "PID_types.h"
#include <math.h>
#include "PID_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include <string.h>
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include <stddef.h>
#include "zero_crossing_types.h"

static void rate_scheduler(RT_MODEL_PID_T *const PID_M);

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(RT_MODEL_PID_T *const PID_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (PID_M->Timing.TaskCounters.TID[1])++;
  if ((PID_M->Timing.TaskCounters.TID[1]) > 99) {// Sample time: [1.0s, 0.0s]
    PID_M->Timing.TaskCounters.TID[1] = 0;
  }
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// Output and update for atomic system:
//    '<Root>/Tranfsform Gains1'
//    '<Root>/Tranfsform Gains2'
//    '<Root>/Tranfsform Gains3'
//
void PID::PID_TranfsformGains1(const real_T rtu_TwistBody[6], const real_T
  rtu_LinearRotm[9], const real_T rtu_AngularRotm[9], real_T
  *rty_WorldFrameGains, real_T *rty_WorldFrameGains_i, real_T
  *rty_WorldFrameGains_f, real_T *rty_WorldFrameGains_o, real_T
  *rty_WorldFrameGains_on, real_T *rty_WorldFrameGains_c,
  B_TranfsformGains1_PID_T *localB, P_TranfsformGains1_PID_T *localP)
{
  real_T rtb_BodyY2;
  real_T rtb_BodyZ2;
  real_T rtb_BodyZ2_ev;
  real_T rtb_DotProduct2;
  real_T rtb_DotProduct2_o4;
  int32_T i;

  // Product: '<S23>/World X Body' incorporates:
  //   Constant: '<S10>/World Principal X'

  localB->YinBody = localP->WorldPrincipalX_Value[1];
  localB->Sqrt1 = localP->WorldPrincipalX_Value[0];
  rtb_DotProduct2 = localP->WorldPrincipalX_Value[2];
  for (i = 0; i < 3; i++) {
    // Abs: '<S23>/Abs1'
    localB->TmpSignalConversionAtDotP_e[i] = fabs((rtu_LinearRotm[i + 3] *
      localB->YinBody + rtu_LinearRotm[i] * localB->Sqrt1) + rtu_LinearRotm[i +
      6] * rtb_DotProduct2);
  }

  // End of Product: '<S23>/World X Body'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // Product: '<S24>/World X Body' incorporates:
    //   Constant: '<S10>/World Principal X'

    localB->YinBody = localP->WorldPrincipalX_Value[1];
    localB->Sqrt1 = localP->WorldPrincipalX_Value[0];
    rtb_DotProduct2 = localP->WorldPrincipalX_Value[2];
    for (i = 0; i < 3; i++) {
      // Abs: '<S24>/Abs1'
      localB->Abs1[i] = fabs((rtu_AngularRotm[i + 3] * localB->YinBody +
        rtu_AngularRotm[i] * localB->Sqrt1) + rtu_AngularRotm[i + 6] *
        rtb_DotProduct2);
    }

    // End of Product: '<S24>/World X Body'
  }

  // Product: '<S23>/Zero' incorporates:
  //   Constant: '<S23>/Constant2'
  //   Constant: '<S23>/Infinity'

  localB->YinBody = localP->Constant2_Value / localP->Infinity_Value;

  // Switch: '<S23>/Non Zero Gains'
  if (rtu_TwistBody[0] > localP->NonZeroGains_Threshold) {
    localB->Sqrt1 = rtu_TwistBody[0];
  } else {
    localB->Sqrt1 = localB->YinBody;
  }

  // End of Switch: '<S23>/Non Zero Gains'

  // Product: '<S23>/X Term' incorporates:
  //   Constant: '<S23>/Constant'
  //   Product: '<S23>/Body X ^2'

  localB->XTerm = localP->Constant_Value / (localB->Sqrt1 * localB->Sqrt1);

  // Switch: '<S23>/Non Zero Gains 1'
  if (rtu_TwistBody[1] > localP->NonZeroGains1_Threshold) {
    localB->Sqrt1 = rtu_TwistBody[1];
  } else {
    localB->Sqrt1 = localB->YinBody;
  }

  // End of Switch: '<S23>/Non Zero Gains 1'

  // Product: '<S23>/Body Y ^2'
  rtb_BodyY2 = localB->Sqrt1 * localB->Sqrt1;

  // Switch: '<S23>/Stop Infinite Tangent'
  if (localB->TmpSignalConversionAtDotP_e[1] >
      localP->StopInfiniteTangent_Threshold) {
    rtb_DotProduct2 = localB->TmpSignalConversionAtDotP_e[1];
  } else {
    rtb_DotProduct2 = localB->YinBody;
  }

  // Product: '<S23>/tan(alpha)' incorporates:
  //   Switch: '<S23>/Stop Infinite Tangent'

  localB->Sqrt1 = localB->TmpSignalConversionAtDotP_e[1] / rtb_DotProduct2;

  // Switch: '<S23>/Account Near Infinite Tangent' incorporates:
  //   Constant: '<S23>/Infinity'

  if (!(localB->Sqrt1 != 0.0)) {
    localB->Sqrt1 = localP->Infinity_Value;
  }

  // End of Switch: '<S23>/Account Near Infinite Tangent'

  // Product: '<S23>/tan(alpha)^2'
  rtb_DotProduct2 = localB->Sqrt1 * localB->Sqrt1;

  // Switch: '<S23>/Non Zero Gains 2'
  if (rtu_TwistBody[2] > localP->NonZeroGains2_Threshold) {
    rtb_DotProduct2_o4 = rtu_TwistBody[2];
  } else {
    rtb_DotProduct2_o4 = localB->YinBody;
  }

  // End of Switch: '<S23>/Non Zero Gains 2'

  // Product: '<S23>/Body Z ^2'
  rtb_BodyZ2 = rtb_DotProduct2_o4 * rtb_DotProduct2_o4;

  // DotProduct: '<S23>/Dot Product'
  rtb_DotProduct2_o4 = localB->TmpSignalConversionAtDotP_e[0] *
    localB->TmpSignalConversionAtDotP_e[0] + localB->
    TmpSignalConversionAtDotP_e[1] * localB->TmpSignalConversionAtDotP_e[1];

  // Switch: '<S23>/Stop Infinite Tanget 2' incorporates:
  //   DotProduct: '<S23>/Dot Product'

  if (rtb_DotProduct2_o4 > localP->StopInfiniteTanget2_Threshold) {
    localB->YinBody = rtb_DotProduct2_o4;
  }

  // Product: '<S23>/tan(beta)' incorporates:
  //   Sqrt: '<S23>/Sqrt'
  //   Switch: '<S23>/Stop Infinite Tanget 2'

  rtb_DotProduct2_o4 = localB->TmpSignalConversionAtDotP_e[2] / sqrt
    (localB->YinBody);

  // Switch: '<S23>/Account Near Infinite Tangent1' incorporates:
  //   Constant: '<S23>/Infinity'

  if (!(rtb_DotProduct2_o4 != 0.0)) {
    rtb_DotProduct2_o4 = localP->Infinity_Value;
  }

  // End of Switch: '<S23>/Account Near Infinite Tangent1'

  // Math: '<S23>/X in Body' incorporates:
  //   Constant: '<S23>/Constant'
  //   Constant: '<S23>/Constant1'
  //   Product: '<S23>/Num Z'
  //   Product: '<S23>/Product'
  //   Product: '<S23>/Y Term'
  //   Product: '<S23>/Z Term'
  //   Sum: '<S23>/Sum'
  //   Sum: '<S23>/Sum1'

  rtb_DotProduct2 = rt_powd_snf((rtb_DotProduct2 + localP->Constant_Value) *
    (rtb_DotProduct2_o4 * rtb_DotProduct2_o4) / rtb_BodyZ2 + (rtb_DotProduct2 /
    rtb_BodyY2 + localB->XTerm), localP->Constant1_Value);

  // Product: '<S23>/Y in Body'
  localB->Sqrt1 *= rtb_DotProduct2;

  // DotProduct: '<S23>/Dot Product1' incorporates:
  //   DotProduct: '<S23>/Dot Product2'
  //   SignalConversion generated from: '<S23>/Dot Product1'

  localB->XTerm = rtb_DotProduct2 * rtb_DotProduct2 + localB->Sqrt1 *
    localB->Sqrt1;

  // SignalConversion generated from: '<S23>/Dot Product2' incorporates:
  //   DotProduct: '<S23>/Dot Product1'
  //   Product: '<S23>/Z in Body'
  //   Sqrt: '<S23>/Sqrt1'

  localB->TmpSignalConversionAtDotP_e[2] = sqrt(localB->XTerm) *
    rtb_DotProduct2_o4;

  // Product: '<S24>/Zero' incorporates:
  //   Constant: '<S24>/Constant2'
  //   Constant: '<S24>/Infinity'

  localB->Zero = localP->Constant2_Value_b / localP->Infinity_Value_d;

  // Switch: '<S24>/Non Zero Gains'
  if (rtu_TwistBody[3] > localP->NonZeroGains_Threshold_n) {
    rtb_DotProduct2_o4 = rtu_TwistBody[3];
  } else {
    rtb_DotProduct2_o4 = localB->Zero;
  }

  // End of Switch: '<S24>/Non Zero Gains'

  // Product: '<S24>/X Term' incorporates:
  //   Constant: '<S24>/Constant'
  //   Product: '<S24>/Body X ^2'

  localB->YinBody = localP->Constant_Value_l / (rtb_DotProduct2_o4 *
    rtb_DotProduct2_o4);

  // Switch: '<S24>/Non Zero Gains 1'
  if (rtu_TwistBody[4] > localP->NonZeroGains1_Threshold_p) {
    rtb_DotProduct2_o4 = rtu_TwistBody[4];
  } else {
    rtb_DotProduct2_o4 = localB->Zero;
  }

  // End of Switch: '<S24>/Non Zero Gains 1'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // Switch: '<S24>/Stop Infinite Tangent'
    if (localB->Abs1[1] > localP->StopInfiniteTangent_Threshold_a) {
      rtb_DotProduct2 = localB->Abs1[1];
    } else {
      rtb_DotProduct2 = localB->Zero;
    }

    // Product: '<S24>/tan(alpha)' incorporates:
    //   Switch: '<S24>/Stop Infinite Tangent'

    localB->Sqrt1 = localB->Abs1[1] / rtb_DotProduct2;

    // Switch: '<S24>/Account Near Infinite Tangent'
    if (localB->Sqrt1 != 0.0) {
      // Switch: '<S24>/Account Near Infinite Tangent'
      localB->AccountNearInfiniteTangent = localB->Sqrt1;
    } else {
      // Switch: '<S24>/Account Near Infinite Tangent' incorporates:
      //   Constant: '<S24>/Infinity'

      localB->AccountNearInfiniteTangent = localP->Infinity_Value_d;
    }

    // End of Switch: '<S24>/Account Near Infinite Tangent'

    // Product: '<S24>/tan(alpha)^2'
    localB->tanalpha2 = localB->AccountNearInfiniteTangent *
      localB->AccountNearInfiniteTangent;
  }

  // Product: '<S24>/Y Term' incorporates:
  //   Product: '<S24>/Body Y ^2'

  rtb_DotProduct2 = localB->tanalpha2 / (rtb_DotProduct2_o4 * rtb_DotProduct2_o4);

  // Switch: '<S24>/Non Zero Gains 2'
  if (rtu_TwistBody[5] > localP->NonZeroGains2_Threshold_k) {
    rtb_DotProduct2_o4 = rtu_TwistBody[5];
  } else {
    rtb_DotProduct2_o4 = localB->Zero;
  }

  // End of Switch: '<S24>/Non Zero Gains 2'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // DotProduct: '<S24>/Dot Product'
    localB->Sqrt1 = localB->Abs1[0] * localB->Abs1[0] + localB->Abs1[1] *
      localB->Abs1[1];

    // Switch: '<S24>/Stop Infinite Tanget 2' incorporates:
    //   DotProduct: '<S24>/Dot Product'

    if (!(localB->Sqrt1 > localP->StopInfiniteTanget2_Threshold_l)) {
      localB->Sqrt1 = localB->Zero;
    }

    // Product: '<S24>/tan(beta)' incorporates:
    //   Sqrt: '<S24>/Sqrt'
    //   Switch: '<S24>/Stop Infinite Tanget 2'

    localB->Sqrt1 = localB->Abs1[2] / sqrt(localB->Sqrt1);

    // Switch: '<S24>/Account Near Infinite Tangent1'
    if (localB->Sqrt1 != 0.0) {
      // Switch: '<S24>/Account Near Infinite Tangent1'
      localB->AccountNearInfiniteTangent1 = localB->Sqrt1;
    } else {
      // Switch: '<S24>/Account Near Infinite Tangent1' incorporates:
      //   Constant: '<S24>/Infinity'

      localB->AccountNearInfiniteTangent1 = localP->Infinity_Value_d;
    }

    // End of Switch: '<S24>/Account Near Infinite Tangent1'

    // Product: '<S24>/Num Z' incorporates:
    //   Constant: '<S24>/Constant'
    //   Product: '<S24>/Product'
    //   Sum: '<S24>/Sum'

    localB->NumZ = (localB->tanalpha2 + localP->Constant_Value_l) *
      (localB->AccountNearInfiniteTangent1 * localB->AccountNearInfiniteTangent1);
  }

  // Math: '<S24>/X in Body' incorporates:
  //   Constant: '<S24>/Constant1'
  //   Product: '<S24>/Body Z ^2'
  //   Product: '<S24>/Z Term'
  //   Sum: '<S24>/Sum1'

  rtb_DotProduct2_o4 = rt_powd_snf(localB->NumZ / (rtb_DotProduct2_o4 *
    rtb_DotProduct2_o4) + (localB->YinBody + rtb_DotProduct2),
    localP->Constant1_Value_i);

  // Product: '<S24>/Y in Body'
  rtb_DotProduct2 = rtb_DotProduct2_o4 * localB->AccountNearInfiniteTangent;

  // DotProduct: '<S24>/Dot Product1' incorporates:
  //   DotProduct: '<S24>/Dot Product2'
  //   SignalConversion generated from: '<S24>/Dot Product1'

  localB->YinBody = rtb_DotProduct2_o4 * rtb_DotProduct2_o4 + rtb_DotProduct2 *
    rtb_DotProduct2;

  // SignalConversion generated from: '<S24>/Dot Product2' incorporates:
  //   DotProduct: '<S24>/Dot Product1'
  //   Product: '<S24>/Z in Body'
  //   Sqrt: '<S24>/Sqrt1'

  localB->TmpSignalConversionAtDot_et[2] = sqrt(localB->YinBody) *
    localB->AccountNearInfiniteTangent1;

  // Sqrt: '<S24>/World Gain' incorporates:
  //   DotProduct: '<S24>/Dot Product2'

  *rty_WorldFrameGains_o = sqrt(localB->TmpSignalConversionAtDot_et[2] *
    localB->TmpSignalConversionAtDot_et[2] + localB->YinBody);

  // Sqrt: '<S23>/World Gain' incorporates:
  //   DotProduct: '<S23>/Dot Product2'

  *rty_WorldFrameGains = sqrt(localB->TmpSignalConversionAtDotP_e[2] *
    localB->TmpSignalConversionAtDotP_e[2] + localB->XTerm);

  // Product: '<S25>/World X Body' incorporates:
  //   Constant: '<S10>/World Principal Y'

  localB->YinBody = localP->WorldPrincipalY_Value[1];
  localB->Sqrt1 = localP->WorldPrincipalY_Value[0];
  rtb_DotProduct2 = localP->WorldPrincipalY_Value[2];
  for (i = 0; i < 3; i++) {
    // Abs: '<S25>/Abs1'
    localB->TmpSignalConversionAtDot_et[i] = fabs((rtu_LinearRotm[i + 3] *
      localB->YinBody + rtu_LinearRotm[i] * localB->Sqrt1) + rtu_LinearRotm[i +
      6] * rtb_DotProduct2);
  }

  // End of Product: '<S25>/World X Body'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // Product: '<S26>/World X Body' incorporates:
    //   Constant: '<S10>/World Principal Y'

    localB->YinBody = localP->WorldPrincipalY_Value[1];
    localB->Sqrt1 = localP->WorldPrincipalY_Value[0];
    rtb_DotProduct2 = localP->WorldPrincipalY_Value[2];
    for (i = 0; i < 3; i++) {
      // Abs: '<S26>/Abs1'
      localB->Abs1[i] = fabs((rtu_AngularRotm[i + 3] * localB->YinBody +
        rtu_AngularRotm[i] * localB->Sqrt1) + rtu_AngularRotm[i + 6] *
        rtb_DotProduct2);
    }

    // End of Product: '<S26>/World X Body'
  }

  // Product: '<S25>/Zero' incorporates:
  //   Constant: '<S25>/Constant2'
  //   Constant: '<S25>/Infinity'

  localB->Sqrt1 = localP->Constant2_Value_f / localP->Infinity_Value_c;

  // Switch: '<S25>/Non Zero Gains'
  if (rtu_TwistBody[0] > localP->NonZeroGains_Threshold_d) {
    localB->YinBody = rtu_TwistBody[0];
  } else {
    localB->YinBody = localB->Sqrt1;
  }

  // End of Switch: '<S25>/Non Zero Gains'

  // Product: '<S25>/X Term' incorporates:
  //   Constant: '<S25>/Constant'
  //   Product: '<S25>/Body X ^2'

  rtb_BodyY2 = localP->Constant_Value_k / (localB->YinBody * localB->YinBody);

  // Switch: '<S25>/Non Zero Gains 1'
  if (rtu_TwistBody[1] > localP->NonZeroGains1_Threshold_m) {
    localB->YinBody = rtu_TwistBody[1];
  } else {
    localB->YinBody = localB->Sqrt1;
  }

  // End of Switch: '<S25>/Non Zero Gains 1'

  // Product: '<S25>/Body Y ^2'
  rtb_BodyZ2 = localB->YinBody * localB->YinBody;

  // Switch: '<S25>/Stop Infinite Tangent'
  if (localB->TmpSignalConversionAtDot_et[1] >
      localP->StopInfiniteTangent_Threshol_au) {
    rtb_DotProduct2 = localB->TmpSignalConversionAtDot_et[1];
  } else {
    rtb_DotProduct2 = localB->Sqrt1;
  }

  // Product: '<S25>/tan(alpha)' incorporates:
  //   Switch: '<S25>/Stop Infinite Tangent'

  localB->YinBody = localB->TmpSignalConversionAtDot_et[1] / rtb_DotProduct2;

  // Switch: '<S25>/Account Near Infinite Tangent' incorporates:
  //   Constant: '<S25>/Infinity'

  if (!(localB->YinBody != 0.0)) {
    localB->YinBody = localP->Infinity_Value_c;
  }

  // End of Switch: '<S25>/Account Near Infinite Tangent'

  // Product: '<S25>/tan(alpha)^2'
  rtb_DotProduct2_o4 = localB->YinBody * localB->YinBody;

  // Switch: '<S25>/Non Zero Gains 2'
  if (rtu_TwistBody[2] > localP->NonZeroGains2_Threshold_o) {
    localB->XTerm = rtu_TwistBody[2];
  } else {
    localB->XTerm = localB->Sqrt1;
  }

  // End of Switch: '<S25>/Non Zero Gains 2'

  // Product: '<S25>/Body Z ^2'
  rtb_BodyZ2_ev = localB->XTerm * localB->XTerm;

  // DotProduct: '<S25>/Dot Product'
  rtb_DotProduct2 = localB->TmpSignalConversionAtDot_et[0] *
    localB->TmpSignalConversionAtDot_et[0] + localB->
    TmpSignalConversionAtDot_et[1] * localB->TmpSignalConversionAtDot_et[1];

  // Switch: '<S25>/Stop Infinite Tanget 2' incorporates:
  //   DotProduct: '<S25>/Dot Product'

  if (rtb_DotProduct2 > localP->StopInfiniteTanget2_Threshold_p) {
    localB->Sqrt1 = rtb_DotProduct2;
  }

  // Product: '<S25>/tan(beta)' incorporates:
  //   Sqrt: '<S25>/Sqrt'
  //   Switch: '<S25>/Stop Infinite Tanget 2'

  localB->XTerm = localB->TmpSignalConversionAtDot_et[2] / sqrt(localB->Sqrt1);

  // Switch: '<S25>/Account Near Infinite Tangent1' incorporates:
  //   Constant: '<S25>/Infinity'

  if (!(localB->XTerm != 0.0)) {
    localB->XTerm = localP->Infinity_Value_c;
  }

  // End of Switch: '<S25>/Account Near Infinite Tangent1'

  // Math: '<S25>/X in Body' incorporates:
  //   Constant: '<S25>/Constant'
  //   Constant: '<S25>/Constant1'
  //   Product: '<S25>/Num Z'
  //   Product: '<S25>/Product'
  //   Product: '<S25>/Y Term'
  //   Product: '<S25>/Z Term'
  //   Sum: '<S25>/Sum'
  //   Sum: '<S25>/Sum1'

  rtb_DotProduct2_o4 = rt_powd_snf((rtb_DotProduct2_o4 +
    localP->Constant_Value_k) * (localB->XTerm * localB->XTerm) / rtb_BodyZ2_ev
    + (rtb_DotProduct2_o4 / rtb_BodyZ2 + rtb_BodyY2), localP->Constant1_Value_l);

  // Product: '<S25>/Y in Body'
  localB->YinBody *= rtb_DotProduct2_o4;

  // DotProduct: '<S25>/Dot Product1' incorporates:
  //   DotProduct: '<S25>/Dot Product2'
  //   SignalConversion generated from: '<S25>/Dot Product1'

  localB->YinBody = rtb_DotProduct2_o4 * rtb_DotProduct2_o4 + localB->YinBody *
    localB->YinBody;

  // SignalConversion generated from: '<S25>/Dot Product2' incorporates:
  //   DotProduct: '<S25>/Dot Product1'
  //   Product: '<S25>/Z in Body'
  //   Sqrt: '<S25>/Sqrt1'

  localB->TmpSignalConversionAtDot_et[2] = sqrt(localB->YinBody) * localB->XTerm;

  // Product: '<S26>/Zero' incorporates:
  //   Constant: '<S26>/Constant2'
  //   Constant: '<S26>/Infinity'

  localB->Zero_f = localP->Constant2_Value_d / localP->Infinity_Value_e;

  // Switch: '<S26>/Non Zero Gains'
  if (rtu_TwistBody[3] > localP->NonZeroGains_Threshold_k) {
    localB->XTerm = rtu_TwistBody[3];
  } else {
    localB->XTerm = localB->Zero_f;
  }

  // End of Switch: '<S26>/Non Zero Gains'

  // Product: '<S26>/X Term' incorporates:
  //   Constant: '<S26>/Constant'
  //   Product: '<S26>/Body X ^2'

  rtb_DotProduct2_o4 = localP->Constant_Value_g / (localB->XTerm * localB->XTerm);

  // Switch: '<S26>/Non Zero Gains 1'
  if (rtu_TwistBody[4] > localP->NonZeroGains1_Threshold_d) {
    localB->XTerm = rtu_TwistBody[4];
  } else {
    localB->XTerm = localB->Zero_f;
  }

  // End of Switch: '<S26>/Non Zero Gains 1'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // Switch: '<S26>/Stop Infinite Tangent'
    if (localB->Abs1[1] > localP->StopInfiniteTangent_Threshold_i) {
      rtb_DotProduct2 = localB->Abs1[1];
    } else {
      rtb_DotProduct2 = localB->Zero_f;
    }

    // Product: '<S26>/tan(alpha)' incorporates:
    //   Switch: '<S26>/Stop Infinite Tangent'

    localB->Sqrt1 = localB->Abs1[1] / rtb_DotProduct2;

    // Switch: '<S26>/Account Near Infinite Tangent'
    if (localB->Sqrt1 != 0.0) {
      // Switch: '<S26>/Account Near Infinite Tangent'
      localB->AccountNearInfiniteTangent_k = localB->Sqrt1;
    } else {
      // Switch: '<S26>/Account Near Infinite Tangent' incorporates:
      //   Constant: '<S26>/Infinity'

      localB->AccountNearInfiniteTangent_k = localP->Infinity_Value_e;
    }

    // End of Switch: '<S26>/Account Near Infinite Tangent'

    // Product: '<S26>/tan(alpha)^2'
    localB->tanalpha2_f = localB->AccountNearInfiniteTangent_k *
      localB->AccountNearInfiniteTangent_k;
  }

  // Product: '<S26>/Y Term' incorporates:
  //   Product: '<S26>/Body Y ^2'

  rtb_DotProduct2 = localB->tanalpha2_f / (localB->XTerm * localB->XTerm);

  // Switch: '<S26>/Non Zero Gains 2'
  if (rtu_TwistBody[5] > localP->NonZeroGains2_Threshold_p) {
    localB->XTerm = rtu_TwistBody[5];
  } else {
    localB->XTerm = localB->Zero_f;
  }

  // End of Switch: '<S26>/Non Zero Gains 2'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // DotProduct: '<S26>/Dot Product'
    localB->Sqrt1 = localB->Abs1[0] * localB->Abs1[0] + localB->Abs1[1] *
      localB->Abs1[1];

    // Switch: '<S26>/Stop Infinite Tanget 2' incorporates:
    //   DotProduct: '<S26>/Dot Product'

    if (!(localB->Sqrt1 > localP->StopInfiniteTanget2_Threshold_g)) {
      localB->Sqrt1 = localB->Zero_f;
    }

    // Product: '<S26>/tan(beta)' incorporates:
    //   Sqrt: '<S26>/Sqrt'
    //   Switch: '<S26>/Stop Infinite Tanget 2'

    localB->Sqrt1 = localB->Abs1[2] / sqrt(localB->Sqrt1);

    // Switch: '<S26>/Account Near Infinite Tangent1'
    if (localB->Sqrt1 != 0.0) {
      // Switch: '<S26>/Account Near Infinite Tangent1'
      localB->AccountNearInfiniteTangent1_e = localB->Sqrt1;
    } else {
      // Switch: '<S26>/Account Near Infinite Tangent1' incorporates:
      //   Constant: '<S26>/Infinity'

      localB->AccountNearInfiniteTangent1_e = localP->Infinity_Value_e;
    }

    // End of Switch: '<S26>/Account Near Infinite Tangent1'

    // Product: '<S26>/Num Z' incorporates:
    //   Constant: '<S26>/Constant'
    //   Product: '<S26>/Product'
    //   Sum: '<S26>/Sum'

    localB->NumZ_g = (localB->tanalpha2_f + localP->Constant_Value_g) *
      (localB->AccountNearInfiniteTangent1_e *
       localB->AccountNearInfiniteTangent1_e);
  }

  // Math: '<S26>/X in Body' incorporates:
  //   Constant: '<S26>/Constant1'
  //   Product: '<S26>/Body Z ^2'
  //   Product: '<S26>/Z Term'
  //   Sum: '<S26>/Sum1'

  localB->XTerm = rt_powd_snf(localB->NumZ_g / (localB->XTerm * localB->XTerm) +
    (rtb_DotProduct2_o4 + rtb_DotProduct2), localP->Constant1_Value_n);

  // Product: '<S26>/Y in Body'
  rtb_DotProduct2_o4 = localB->XTerm * localB->AccountNearInfiniteTangent_k;

  // DotProduct: '<S26>/Dot Product1' incorporates:
  //   DotProduct: '<S26>/Dot Product2'
  //   SignalConversion generated from: '<S26>/Dot Product1'

  localB->XTerm = localB->XTerm * localB->XTerm + rtb_DotProduct2_o4 *
    rtb_DotProduct2_o4;

  // SignalConversion generated from: '<S26>/Dot Product2' incorporates:
  //   DotProduct: '<S26>/Dot Product1'
  //   Product: '<S26>/Z in Body'
  //   Sqrt: '<S26>/Sqrt1'

  localB->TmpSignalConversionAtDotP_e[2] = sqrt(localB->XTerm) *
    localB->AccountNearInfiniteTangent1_e;

  // Sqrt: '<S26>/World Gain' incorporates:
  //   DotProduct: '<S26>/Dot Product2'

  *rty_WorldFrameGains_on = sqrt(localB->TmpSignalConversionAtDotP_e[2] *
    localB->TmpSignalConversionAtDotP_e[2] + localB->XTerm);

  // Sqrt: '<S25>/World Gain' incorporates:
  //   DotProduct: '<S25>/Dot Product2'

  *rty_WorldFrameGains_i = sqrt(localB->TmpSignalConversionAtDot_et[2] *
    localB->TmpSignalConversionAtDot_et[2] + localB->YinBody);

  // Product: '<S27>/World X Body' incorporates:
  //   Constant: '<S10>/World Principal Z'

  localB->YinBody = localP->WorldPrincipalZ_Value[1];
  localB->Sqrt1 = localP->WorldPrincipalZ_Value[0];
  rtb_DotProduct2 = localP->WorldPrincipalZ_Value[2];
  for (i = 0; i < 3; i++) {
    // Abs: '<S27>/Abs1'
    localB->TmpSignalConversionAtDot_et[i] = fabs((rtu_LinearRotm[i + 3] *
      localB->YinBody + rtu_LinearRotm[i] * localB->Sqrt1) + rtu_LinearRotm[i +
      6] * rtb_DotProduct2);
  }

  // End of Product: '<S27>/World X Body'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // Product: '<S28>/World X Body' incorporates:
    //   Constant: '<S10>/World Principal Z'

    localB->YinBody = localP->WorldPrincipalZ_Value[1];
    localB->Sqrt1 = localP->WorldPrincipalZ_Value[0];
    rtb_DotProduct2 = localP->WorldPrincipalZ_Value[2];
    for (i = 0; i < 3; i++) {
      // Abs: '<S28>/Abs1'
      localB->Abs1[i] = fabs((rtu_AngularRotm[i + 3] * localB->YinBody +
        rtu_AngularRotm[i] * localB->Sqrt1) + rtu_AngularRotm[i + 6] *
        rtb_DotProduct2);
    }

    // End of Product: '<S28>/World X Body'
  }

  // Product: '<S27>/Zero' incorporates:
  //   Constant: '<S27>/Constant2'
  //   Constant: '<S27>/Infinity'

  localB->Sqrt1 = localP->Constant2_Value_k / localP->Infinity_Value_h;

  // Switch: '<S27>/Non Zero Gains'
  if (rtu_TwistBody[0] > localP->NonZeroGains_Threshold_o) {
    localB->YinBody = rtu_TwistBody[0];
  } else {
    localB->YinBody = localB->Sqrt1;
  }

  // End of Switch: '<S27>/Non Zero Gains'

  // Product: '<S27>/X Term' incorporates:
  //   Constant: '<S27>/Constant'
  //   Product: '<S27>/Body X ^2'

  rtb_BodyY2 = localP->Constant_Value_m / (localB->YinBody * localB->YinBody);

  // Switch: '<S27>/Non Zero Gains 1'
  if (rtu_TwistBody[1] > localP->NonZeroGains1_Threshold_f) {
    localB->YinBody = rtu_TwistBody[1];
  } else {
    localB->YinBody = localB->Sqrt1;
  }

  // End of Switch: '<S27>/Non Zero Gains 1'

  // Product: '<S27>/Body Y ^2'
  rtb_BodyZ2 = localB->YinBody * localB->YinBody;

  // Switch: '<S27>/Stop Infinite Tangent'
  if (localB->TmpSignalConversionAtDot_et[1] >
      localP->StopInfiniteTangent_Threshold_n) {
    rtb_DotProduct2 = localB->TmpSignalConversionAtDot_et[1];
  } else {
    rtb_DotProduct2 = localB->Sqrt1;
  }

  // Product: '<S27>/tan(alpha)' incorporates:
  //   Switch: '<S27>/Stop Infinite Tangent'

  localB->YinBody = localB->TmpSignalConversionAtDot_et[1] / rtb_DotProduct2;

  // Switch: '<S27>/Account Near Infinite Tangent' incorporates:
  //   Constant: '<S27>/Infinity'

  if (!(localB->YinBody != 0.0)) {
    localB->YinBody = localP->Infinity_Value_h;
  }

  // End of Switch: '<S27>/Account Near Infinite Tangent'

  // Product: '<S27>/tan(alpha)^2'
  rtb_DotProduct2_o4 = localB->YinBody * localB->YinBody;

  // Switch: '<S27>/Non Zero Gains 2'
  if (rtu_TwistBody[2] > localP->NonZeroGains2_Threshold_kl) {
    localB->XTerm = rtu_TwistBody[2];
  } else {
    localB->XTerm = localB->Sqrt1;
  }

  // End of Switch: '<S27>/Non Zero Gains 2'

  // Product: '<S27>/Body Z ^2'
  rtb_BodyZ2_ev = localB->XTerm * localB->XTerm;

  // DotProduct: '<S27>/Dot Product'
  rtb_DotProduct2 = localB->TmpSignalConversionAtDot_et[0] *
    localB->TmpSignalConversionAtDot_et[0] + localB->
    TmpSignalConversionAtDot_et[1] * localB->TmpSignalConversionAtDot_et[1];

  // Switch: '<S27>/Stop Infinite Tanget 2' incorporates:
  //   DotProduct: '<S27>/Dot Product'

  if (rtb_DotProduct2 > localP->StopInfiniteTanget2_Threshold_a) {
    localB->Sqrt1 = rtb_DotProduct2;
  }

  // Product: '<S27>/tan(beta)' incorporates:
  //   Sqrt: '<S27>/Sqrt'
  //   Switch: '<S27>/Stop Infinite Tanget 2'

  localB->XTerm = localB->TmpSignalConversionAtDot_et[2] / sqrt(localB->Sqrt1);

  // Switch: '<S27>/Account Near Infinite Tangent1' incorporates:
  //   Constant: '<S27>/Infinity'

  if (!(localB->XTerm != 0.0)) {
    localB->XTerm = localP->Infinity_Value_h;
  }

  // End of Switch: '<S27>/Account Near Infinite Tangent1'

  // Math: '<S27>/X in Body' incorporates:
  //   Constant: '<S27>/Constant'
  //   Constant: '<S27>/Constant1'
  //   Product: '<S27>/Num Z'
  //   Product: '<S27>/Product'
  //   Product: '<S27>/Y Term'
  //   Product: '<S27>/Z Term'
  //   Sum: '<S27>/Sum'
  //   Sum: '<S27>/Sum1'

  rtb_DotProduct2_o4 = rt_powd_snf((rtb_DotProduct2_o4 +
    localP->Constant_Value_m) * (localB->XTerm * localB->XTerm) / rtb_BodyZ2_ev
    + (rtb_DotProduct2_o4 / rtb_BodyZ2 + rtb_BodyY2), localP->Constant1_Value_k);

  // Product: '<S27>/Y in Body'
  localB->YinBody *= rtb_DotProduct2_o4;

  // DotProduct: '<S27>/Dot Product1' incorporates:
  //   DotProduct: '<S27>/Dot Product2'
  //   SignalConversion generated from: '<S27>/Dot Product1'

  localB->YinBody = rtb_DotProduct2_o4 * rtb_DotProduct2_o4 + localB->YinBody *
    localB->YinBody;

  // SignalConversion generated from: '<S27>/Dot Product2' incorporates:
  //   DotProduct: '<S27>/Dot Product1'
  //   Product: '<S27>/Z in Body'
  //   Sqrt: '<S27>/Sqrt1'

  localB->TmpSignalConversionAtDot_et[2] = sqrt(localB->YinBody) * localB->XTerm;

  // Product: '<S28>/Zero' incorporates:
  //   Constant: '<S28>/Constant2'
  //   Constant: '<S28>/Infinity'

  localB->Zero_h = localP->Constant2_Value_g / localP->Infinity_Value_d4;

  // Switch: '<S28>/Non Zero Gains'
  if (rtu_TwistBody[3] > localP->NonZeroGains_Threshold_f) {
    localB->XTerm = rtu_TwistBody[3];
  } else {
    localB->XTerm = localB->Zero_h;
  }

  // End of Switch: '<S28>/Non Zero Gains'

  // Product: '<S28>/X Term' incorporates:
  //   Constant: '<S28>/Constant'
  //   Product: '<S28>/Body X ^2'

  rtb_DotProduct2_o4 = localP->Constant_Value_d / (localB->XTerm * localB->XTerm);

  // Switch: '<S28>/Non Zero Gains 1'
  if (rtu_TwistBody[4] > localP->NonZeroGains1_Threshold_b) {
    localB->XTerm = rtu_TwistBody[4];
  } else {
    localB->XTerm = localB->Zero_h;
  }

  // End of Switch: '<S28>/Non Zero Gains 1'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // Switch: '<S28>/Stop Infinite Tangent'
    if (localB->Abs1[1] > localP->StopInfiniteTangent_Threshold_m) {
      rtb_DotProduct2 = localB->Abs1[1];
    } else {
      rtb_DotProduct2 = localB->Zero_h;
    }

    // Product: '<S28>/tan(alpha)' incorporates:
    //   Switch: '<S28>/Stop Infinite Tangent'

    localB->Sqrt1 = localB->Abs1[1] / rtb_DotProduct2;

    // Switch: '<S28>/Account Near Infinite Tangent'
    if (localB->Sqrt1 != 0.0) {
      // Switch: '<S28>/Account Near Infinite Tangent'
      localB->AccountNearInfiniteTangent_m = localB->Sqrt1;
    } else {
      // Switch: '<S28>/Account Near Infinite Tangent' incorporates:
      //   Constant: '<S28>/Infinity'

      localB->AccountNearInfiniteTangent_m = localP->Infinity_Value_d4;
    }

    // End of Switch: '<S28>/Account Near Infinite Tangent'

    // Product: '<S28>/tan(alpha)^2'
    localB->tanalpha2_b = localB->AccountNearInfiniteTangent_m *
      localB->AccountNearInfiniteTangent_m;
  }

  // Product: '<S28>/Y Term' incorporates:
  //   Product: '<S28>/Body Y ^2'

  rtb_DotProduct2 = localB->tanalpha2_b / (localB->XTerm * localB->XTerm);

  // Switch: '<S28>/Non Zero Gains 2'
  if (rtu_TwistBody[5] > localP->NonZeroGains2_Threshold_a) {
    localB->XTerm = rtu_TwistBody[5];
  } else {
    localB->XTerm = localB->Zero_h;
  }

  // End of Switch: '<S28>/Non Zero Gains 2'
  if ((&PID_M)->Timing.TaskCounters.TID[1] == 0) {
    // DotProduct: '<S28>/Dot Product'
    localB->Sqrt1 = localB->Abs1[0] * localB->Abs1[0] + localB->Abs1[1] *
      localB->Abs1[1];

    // Switch: '<S28>/Stop Infinite Tanget 2' incorporates:
    //   DotProduct: '<S28>/Dot Product'

    if (!(localB->Sqrt1 > localP->StopInfiniteTanget2_Threshol_gv)) {
      localB->Sqrt1 = localB->Zero_h;
    }

    // Product: '<S28>/tan(beta)' incorporates:
    //   Sqrt: '<S28>/Sqrt'
    //   Switch: '<S28>/Stop Infinite Tanget 2'

    localB->Sqrt1 = localB->Abs1[2] / sqrt(localB->Sqrt1);

    // Switch: '<S28>/Account Near Infinite Tangent1'
    if (localB->Sqrt1 != 0.0) {
      // Switch: '<S28>/Account Near Infinite Tangent1'
      localB->AccountNearInfiniteTangent1_n = localB->Sqrt1;
    } else {
      // Switch: '<S28>/Account Near Infinite Tangent1' incorporates:
      //   Constant: '<S28>/Infinity'

      localB->AccountNearInfiniteTangent1_n = localP->Infinity_Value_d4;
    }

    // End of Switch: '<S28>/Account Near Infinite Tangent1'

    // Product: '<S28>/Num Z' incorporates:
    //   Constant: '<S28>/Constant'
    //   Product: '<S28>/Product'
    //   Sum: '<S28>/Sum'

    localB->NumZ_gv = (localB->tanalpha2_b + localP->Constant_Value_d) *
      (localB->AccountNearInfiniteTangent1_n *
       localB->AccountNearInfiniteTangent1_n);
  }

  // Math: '<S28>/X in Body' incorporates:
  //   Constant: '<S28>/Constant1'
  //   Product: '<S28>/Body Z ^2'
  //   Product: '<S28>/Z Term'
  //   Sum: '<S28>/Sum1'

  localB->XTerm = rt_powd_snf(localB->NumZ_gv / (localB->XTerm * localB->XTerm)
    + (rtb_DotProduct2_o4 + rtb_DotProduct2), localP->Constant1_Value_e);

  // Product: '<S28>/Y in Body'
  rtb_DotProduct2_o4 = localB->XTerm * localB->AccountNearInfiniteTangent_m;

  // DotProduct: '<S28>/Dot Product1' incorporates:
  //   DotProduct: '<S28>/Dot Product2'
  //   SignalConversion generated from: '<S28>/Dot Product1'

  localB->XTerm = localB->XTerm * localB->XTerm + rtb_DotProduct2_o4 *
    rtb_DotProduct2_o4;

  // SignalConversion generated from: '<S28>/Dot Product2' incorporates:
  //   DotProduct: '<S28>/Dot Product1'
  //   Product: '<S28>/Z in Body'
  //   Sqrt: '<S28>/Sqrt1'

  localB->TmpSignalConversionAtDotP_e[2] = sqrt(localB->XTerm) *
    localB->AccountNearInfiniteTangent1_n;

  // Sqrt: '<S28>/World Gain' incorporates:
  //   DotProduct: '<S28>/Dot Product2'

  *rty_WorldFrameGains_c = sqrt(localB->TmpSignalConversionAtDotP_e[2] *
    localB->TmpSignalConversionAtDotP_e[2] + localB->XTerm);

  // Sqrt: '<S27>/World Gain' incorporates:
  //   DotProduct: '<S27>/Dot Product2'

  *rty_WorldFrameGains_f = sqrt(localB->TmpSignalConversionAtDot_et[2] *
    localB->TmpSignalConversionAtDot_et[2] + localB->YinBody);
}

// Function for MATLAB Function: '<S1>/Check if gains change'
boolean_T PID::PID_isequal(const real_T varargin_1[6], const real_T varargin_2[6])
{
  boolean_T b_p;
  boolean_T exitg1;
  boolean_T p;
  p = false;
  b_p = true;
  PID_B.k = 0;
  exitg1 = false;
  while ((!exitg1) && (PID_B.k < 6)) {
    if (!(varargin_1[PID_B.k] == varargin_2[PID_B.k])) {
      b_p = false;
      exitg1 = true;
    } else {
      PID_B.k++;
    }
  }

  if (b_p) {
    p = true;
  }

  return p;
}

// Function for MATLAB Function: '<S2>/Normalize Error'
void PID::PID_quatmultiply(const real_T q[4], const real_T r[4], real_T qout[4])
{
  qout[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qout[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - r[2] * q[3]);
  qout[2] = (q[0] * r[2] + r[0] * q[2]) + (r[1] * q[3] - q[1] * r[3]);
  qout[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - r[1] * q[2]);
}

void PID::PID_SystemCore_setup(ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[35] = { '/', 't', 'a', 'l', 'o', 's', '/', 'c', 'o',
    'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '/', 'a', 'c', 't', 'i', 'v', 'e',
    '_', 'b', 'o', 'd', 'y', '_', 'f', 'o', 'r', 'c', 'e' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 35; i++) {
    PID_B.b_zeroDelimTopic[i] = tmp[i];
  }

  PID_B.b_zeroDelimTopic[35] = '\x00';
  Pub_PID_53.createPublisher(&PID_B.b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void PID::PID_SystemCore_setup_n(ros_slros2_internal_block_Sub_T *obj)
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
    PID_B.b_zeroDelimTopic_g[i] = tmp[i];
  }

  PID_B.b_zeroDelimTopic_g[24] = '\x00';
  Sub_PID_33.createSubscriber(&PID_B.b_zeroDelimTopic_g[0], qos_profile);
  obj->isSetupComplete = true;
}

void PID::PID_SystemCore_setup_nt(ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[26] = { '/', 't', 'a', 'l', 'o', 's', '/', 'p', 'i',
    'd', '/', 't', 'a', 'r', 'g', 'e', 't', '_', 'p', 'o', 's', 'i', 't', 'i',
    'o', 'n' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 26; i++) {
    PID_B.b_zeroDelimTopic_f[i] = tmp[i];
  }

  PID_B.b_zeroDelimTopic_f[26] = '\x00';
  Sub_PID_39.createSubscriber(&PID_B.b_zeroDelimTopic_f[0], qos_profile);
  obj->isSetupComplete = true;
}

void PID::PID_SystemCore_setup_ntv(ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[32] = { '/', 't', 'a', 'l', 'o', 's', '/', 'c', 'o',
    'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '/', 'm', 'o', 't', 'i', 'o', 'n',
    '_', 'e', 'n', 'a', 'b', 'l', 'e', 'd' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 32; i++) {
    PID_B.b_zeroDelimTopic_b[i] = tmp[i];
  }

  PID_B.b_zeroDelimTopic_b[32] = '\x00';
  Sub_PID_2028.createSubscriber(&PID_B.b_zeroDelimTopic_b[0], qos_profile);
  obj->isSetupComplete = true;
}

// Model step function
void PID::step()
{
  SL_Bus_std_msgs_Bool b_varargout_2;
  boolean_T b_varargout_1;
  boolean_T b_varargout_1_0;
  static const real_T b[4] = { 1.0, 0.0, 0.0, 0.0 };

  // MATLABSystem: '<Root>/Get Parameter1'
  ParamGet_PID_44.getParameter(&PID_B.b_value_p);

  // MATLABSystem: '<Root>/max_control'
  ParamGet_PID_123.getParameter(6U, &PID_B.b_value[0], &PID_B.len);

  // Product: '<Root>/Divide4' incorporates:
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   DataTypeConversion: '<Root>/Force To Double5'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/max_control'

  for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
    PID_B.Divide4[PID_B.i] = static_cast<real_T>(PID_B.b_value[PID_B.i]) /
      static_cast<real_T>(PID_B.b_value_p);
  }

  // End of Product: '<Root>/Divide4'

  // MATLABSystem: '<S6>/SourceBlock'
  b_varargout_1 = Sub_PID_33.getLatestMessage(&PID_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S20>/Enable'

  if (b_varargout_1) {
    // SignalConversion generated from: '<S20>/In1'
    PID_B.In1 = PID_B.b_varargout_2;
  }

  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   SignalConversion generated from: '<S4>/ SFunction '

  PID_B.WorldGain_a = 1.0 / sqrt(((PID_B.In1.pose.pose.orientation.w *
    PID_B.In1.pose.pose.orientation.w + PID_B.In1.pose.pose.orientation.x *
    PID_B.In1.pose.pose.orientation.x) + PID_B.In1.pose.pose.orientation.y *
    PID_B.In1.pose.pose.orientation.y) + PID_B.In1.pose.pose.orientation.z *
    PID_B.In1.pose.pose.orientation.z);
  PID_B.q_idx_0 = PID_B.In1.pose.pose.orientation.w * PID_B.WorldGain_a;
  PID_B.q_idx_1 = PID_B.In1.pose.pose.orientation.x * PID_B.WorldGain_a;
  PID_B.q_idx_2 = PID_B.In1.pose.pose.orientation.y * PID_B.WorldGain_a;
  PID_B.q_idx_3 = PID_B.In1.pose.pose.orientation.z * PID_B.WorldGain_a;
  PID_B.WorldGain_a = PID_B.q_idx_3 * PID_B.q_idx_3;
  PID_B.WorldGain_m = PID_B.q_idx_2 * PID_B.q_idx_2;
  PID_B.WorldGain_p = PID_B.q_idx_1 * PID_B.q_idx_2;
  PID_B.WorldGain_g = PID_B.q_idx_0 * PID_B.q_idx_3;
  PID_B.Divide8 = PID_B.q_idx_1 * PID_B.q_idx_3;
  PID_B.WorldGain_e = PID_B.q_idx_0 * PID_B.q_idx_2;
  PID_B.WorldGain_d = PID_B.q_idx_1 * PID_B.q_idx_1;
  PID_B.q_idx_2 *= PID_B.q_idx_3;
  PID_B.q_idx_0 *= PID_B.q_idx_1;
  PID_B.world2BodyRotm[0] = 1.0 - (PID_B.WorldGain_m + PID_B.WorldGain_a) * 2.0;
  PID_B.world2BodyRotm[1] = (PID_B.WorldGain_p - PID_B.WorldGain_g) * 2.0;
  PID_B.world2BodyRotm[2] = (PID_B.Divide8 + PID_B.WorldGain_e) * 2.0;
  PID_B.world2BodyRotm[3] = (PID_B.WorldGain_p + PID_B.WorldGain_g) * 2.0;
  PID_B.world2BodyRotm[4] = 1.0 - (PID_B.WorldGain_d + PID_B.WorldGain_a) * 2.0;
  PID_B.world2BodyRotm[5] = (PID_B.q_idx_2 - PID_B.q_idx_0) * 2.0;
  PID_B.world2BodyRotm[6] = (PID_B.Divide8 - PID_B.WorldGain_e) * 2.0;
  PID_B.world2BodyRotm[7] = (PID_B.q_idx_2 + PID_B.q_idx_0) * 2.0;
  PID_B.world2BodyRotm[8] = 1.0 - (PID_B.WorldGain_d + PID_B.WorldGain_m) * 2.0;
  memcpy(&PID_B.Body2World[0], &PID_B.world2BodyRotm[0], 9U * sizeof(real_T));
  for (PID_B.i = 0; PID_B.i < 3; PID_B.i++) {
    PID_B.rtb_world2BodyRotm_tmp = (static_cast<int8_T>(PID_B.i + 1) - 1) * 3;
    PID_B.world2BodyRotm[static_cast<int8_T>(PID_B.i + 1) - 1] =
      PID_B.Body2World[PID_B.rtb_world2BodyRotm_tmp];
    PID_B.world2BodyRotm[static_cast<int8_T>(PID_B.i + 1) + 2] =
      PID_B.Body2World[PID_B.rtb_world2BodyRotm_tmp + 1];
    PID_B.world2BodyRotm[static_cast<int8_T>(PID_B.i + 1) + 5] =
      PID_B.Body2World[PID_B.rtb_world2BodyRotm_tmp + 2];
  }

  for (PID_B.i = 0; PID_B.i < 3; PID_B.i++) {
    PID_B.Body2World[3 * PID_B.i] = PID_B.world2BodyRotm[PID_B.i];
    PID_B.Body2World[3 * PID_B.i + 1] = PID_B.world2BodyRotm[PID_B.i + 3];
    PID_B.Body2World[3 * PID_B.i + 2] = PID_B.world2BodyRotm[PID_B.i + 6];
  }

  memcpy(&PID_B.world2BodyRotm[0], &PID_B.Body2World[0], 9U * sizeof(real_T));

  // Math: '<Root>/Body 2 World'
  for (PID_B.i = 0; PID_B.i < 3; PID_B.i++) {
    PID_B.Body2World[3 * PID_B.i] = PID_B.world2BodyRotm[PID_B.i];
    PID_B.Body2World[3 * PID_B.i + 1] = PID_B.world2BodyRotm[PID_B.i + 3];
    PID_B.Body2World[3 * PID_B.i + 2] = PID_B.world2BodyRotm[PID_B.i + 6];
  }

  // End of Math: '<Root>/Body 2 World'

  // Math: '<Root>/Transpose' incorporates:
  //   Math: '<Root>/Body 2 World'

  for (PID_B.i = 0; PID_B.i < 3; PID_B.i++) {
    PID_B.Transpose[3 * PID_B.i] = PID_B.Body2World[PID_B.i];
    PID_B.Transpose[3 * PID_B.i + 1] = PID_B.Body2World[PID_B.i + 3];
    PID_B.Transpose[3 * PID_B.i + 2] = PID_B.Body2World[PID_B.i + 6];
  }

  // End of Math: '<Root>/Transpose'

  // Outputs for Atomic SubSystem: '<Root>/Tranfsform Gains3'
  PID_TranfsformGains1(PID_B.Divide4, PID_B.Transpose, PID_B.IdentityMatrix,
                       &PID_B.WorldGain_g, &PID_B.WorldGain_m,
                       &PID_B.WorldGain_p, &PID_B.q_idx_0, &PID_B.q_idx_1,
                       &PID_B.WorldGain_a, &PID_B.TranfsformGains3,
                       &PID_P.TranfsformGains3);

  // End of Outputs for SubSystem: '<Root>/Tranfsform Gains3'

  // MATLABSystem: '<Root>/surface gain floor'
  ParamGet_PID_240.getParameter(&PID_B.b_value_l);

  // Product: '<Root>/Divide8' incorporates:
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   DataTypeConversion: '<Root>/Force To Double9'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/surface gain floor'

  PID_B.Divide8 = static_cast<real_T>(PID_B.b_value_l) / static_cast<real_T>
    (PID_B.b_value_p);

  // MATLABSystem: '<Root>/surface gain buffer'
  ParamGet_PID_239.getParameter(&PID_B.b_value_l);

  // Switch: '<S9>/If Force Default Gains' incorporates:
  //   Constant: '<S9>/Constant'
  //   Sum: '<S9>/Subtract'

  if (!(PID_B.In1.pose.pose.orientation.z - PID_B.Divide8 >
        PID_P.IfForceDefaultGains_Threshold)) {
    PID_DW.PreviousGainState_PreviousInput = PID_P.Constant_Value_mr;
  }

  // Switch: '<S9>/If Force Surface Gains' incorporates:
  //   Constant: '<S9>/Constant1'
  //   DataTypeConversion: '<Root>/Force To Double10'
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/surface gain buffer'
  //   Product: '<Root>/Divide9'
  //   Sum: '<S9>/Min Default Gains Depth'
  //   Sum: '<S9>/Subtract1'
  //   Switch: '<S9>/If Force Default Gains'

  if (PID_DW.PreviousGainState_PreviousInput >
      PID_P.IfForceSurfaceGains_Threshold) {
    PID_B.IfForceSurfaceGains = PID_P.Constant1_Value;
  } else {
    PID_B.IfForceSurfaceGains = PID_B.In1.pose.pose.orientation.z - (
      static_cast<real_T>(PID_B.b_value_l) / static_cast<real_T>(PID_B.b_value_p)
      + PID_B.Divide8);
  }

  // End of Switch: '<S9>/If Force Surface Gains'

  // MATLABSystem: '<Root>/p surface'
  ParamGet_PID_235.getParameter(6U, &PID_B.b_value[0], &PID_B.len);

  // MATLABSystem: '<Root>/p Gains'
  ParamGet_PID_48.getParameter(6U, &PID_B.b_value_c[0], &PID_B.len);

  // Switch: '<S9>/If Surface Gains2' incorporates:
  //   DataTypeConversion: '<Root>/Force To Double'
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   DataTypeConversion: '<Root>/Force To Double6'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/p Gains'
  //   MATLABSystem: '<Root>/p surface'
  //   Product: '<Root>/Divide3'
  //   Product: '<Root>/Divide5'

  if (PID_B.IfForceSurfaceGains > PID_P.IfSurfaceGains2_Threshold) {
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      PID_B.Divide4[PID_B.i] = static_cast<real_T>(PID_B.b_value[PID_B.i]) /
        static_cast<real_T>(PID_B.b_value_p);
    }
  } else {
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      PID_B.Divide4[PID_B.i] = static_cast<real_T>(PID_B.b_value_c[PID_B.i]) /
        static_cast<real_T>(PID_B.b_value_p);
    }
  }

  // End of Switch: '<S9>/If Surface Gains2'

  // Outputs for Atomic SubSystem: '<Root>/Tranfsform Gains2'
  PID_TranfsformGains1(PID_B.Divide4, PID_B.Transpose, PID_B.IdentityMatrix,
                       &PID_B.WorldGain_e, &PID_B.WorldGain_d, &PID_B.q_idx_2,
                       &PID_B.q_idx_3, &PID_B.WorldGain_eo, &PID_B.WorldGain_pm,
                       &PID_B.TranfsformGains2, &PID_P.TranfsformGains2);

  // End of Outputs for SubSystem: '<Root>/Tranfsform Gains2'

  // MATLABSystem: '<Root>/d surface'
  ParamGet_PID_233.getParameter(6U, &PID_B.b_value[0], &PID_B.len);

  // MATLABSystem: '<Root>/d Gains'
  ParamGet_PID_47.getParameter(6U, &PID_B.b_value_c[0], &PID_B.len);

  // Switch: '<S9>/If Surface Gains1' incorporates:
  //   DataTypeConversion: '<Root>/Force To Double1'
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   DataTypeConversion: '<Root>/Force To Double7'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/d Gains'
  //   MATLABSystem: '<Root>/d surface'
  //   Product: '<Root>/Divide2'
  //   Product: '<Root>/Divide6'

  if (PID_B.IfForceSurfaceGains > PID_P.IfSurfaceGains1_Threshold) {
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      PID_B.IfSurfaceGains1[PID_B.i] = static_cast<real_T>(PID_B.b_value[PID_B.i])
        / static_cast<real_T>(PID_B.b_value_p);
    }
  } else {
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      PID_B.IfSurfaceGains1[PID_B.i] = static_cast<real_T>
        (PID_B.b_value_c[PID_B.i]) / static_cast<real_T>(PID_B.b_value_p);
    }
  }

  // End of Switch: '<S9>/If Surface Gains1'

  // Outputs for Atomic SubSystem: '<Root>/Tranfsform Gains1'
  PID_TranfsformGains1(PID_B.IfSurfaceGains1, PID_B.Transpose,
                       PID_B.IdentityMatrix, &PID_B.WorldGain_p3,
                       &PID_B.WorldGain_g2, &PID_B.WorldGain_l,
                       &PID_B.WorldGain_ad, &PID_B.WorldGain_c,
                       &PID_B.WorldGain_px, &PID_B.TranfsformGains1,
                       &PID_P.TranfsformGains1);

  // End of Outputs for SubSystem: '<Root>/Tranfsform Gains1'

  // MATLABSystem: '<S7>/SourceBlock'
  b_varargout_1_0 = Sub_PID_39.getLatestMessage(&PID_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S21>/Enable'

  if (b_varargout_1_0) {
    // SignalConversion generated from: '<S21>/In1'
    PID_B.In1_c = PID_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S7>/SourceBlock'
  // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'

  // MATLABSystem: '<Root>/Get Parameter'
  ParamGet_PID_43.getParameter(&PID_B.b_value_l);

  // Product: '<Root>/Divide' incorporates:
  //   DataTypeConversion: '<Root>/Force To Double3'
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   MATLABSystem: '<Root>/Get Parameter'
  //   MATLABSystem: '<Root>/Get Parameter1'

  PID_B.Divide = static_cast<real_T>(PID_B.b_value_l) / static_cast<real_T>
    (PID_B.b_value_p);

  // MATLABSystem: '<Root>/Current Time'
  currentROS2TimeDouble(&PID_B.Divide8);

  // MATLABSystem: '<Root>/i surface'
  ParamGet_PID_234.getParameter(6U, &PID_B.b_value[0], &PID_B.len);

  // MATLABSystem: '<Root>/i Gains'
  ParamGet_PID_46.getParameter(6U, &PID_B.b_value_c[0], &PID_B.len);

  // Switch: '<S9>/If Surface Gains' incorporates:
  //   DataTypeConversion: '<Root>/Force To Double2'
  //   DataTypeConversion: '<Root>/Force To Double4'
  //   DataTypeConversion: '<Root>/Force To Double8'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/i Gains'
  //   MATLABSystem: '<Root>/i surface'
  //   Product: '<Root>/Divide1'
  //   Product: '<Root>/Divide7'

  if (PID_B.IfForceSurfaceGains > PID_P.IfSurfaceGains_Threshold) {
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      PID_B.IfSurfaceGains[PID_B.i] = static_cast<real_T>(PID_B.b_value[PID_B.i])
        / static_cast<real_T>(PID_B.b_value_p);
    }
  } else {
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      PID_B.IfSurfaceGains[PID_B.i] = static_cast<real_T>
        (PID_B.b_value_c[PID_B.i]) / static_cast<real_T>(PID_B.b_value_p);
    }
  }

  // End of Switch: '<S9>/If Surface Gains'

  // Outputs for Triggered SubSystem: '<Root>/Check if Gains Have Changed' incorporates:
  //   TriggerPort: '<S1>/Trigger'

  // MATLABSystem: '<S6>/SourceBlock'
  if (b_varargout_1 && (PID_PrevZCX.CheckifGainsHaveChanged_Trig_ZC != POS_ZCSIG))
  {
    // MATLAB Function: '<S1>/Check if gains change' incorporates:
    //   Memory: '<S1>/Previous D'
    //   Memory: '<S1>/Previous I'
    //   Memory: '<S1>/Previous P'
    //   Switch: '<S9>/If Surface Gains'
    //   Switch: '<S9>/If Surface Gains1'
    //   Switch: '<S9>/If Surface Gains2'

    if (!PID_isequal(PID_B.Divide4, PID_DW.PreviousP_PreviousInput)) {
      PID_B.haveGainsChanged = true;
    } else if (!PID_isequal(PID_B.IfSurfaceGains, PID_DW.PreviousI_PreviousInput))
    {
      PID_B.haveGainsChanged = true;
    } else {
      PID_B.haveGainsChanged = !PID_isequal(PID_B.IfSurfaceGains1,
        PID_DW.PreviousD_PreviousInput);
    }

    // End of MATLAB Function: '<S1>/Check if gains change'
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      // Update for Memory: '<S1>/Previous P' incorporates:
      //   Switch: '<S9>/If Surface Gains2'

      PID_DW.PreviousP_PreviousInput[PID_B.i] = PID_B.Divide4[PID_B.i];

      // Update for Memory: '<S1>/Previous I' incorporates:
      //   Switch: '<S9>/If Surface Gains'

      PID_DW.PreviousI_PreviousInput[PID_B.i] = PID_B.IfSurfaceGains[PID_B.i];

      // Update for Memory: '<S1>/Previous D' incorporates:
      //   Switch: '<S9>/If Surface Gains1'

      PID_DW.PreviousD_PreviousInput[PID_B.i] = PID_B.IfSurfaceGains1[PID_B.i];
    }
  }

  PID_PrevZCX.CheckifGainsHaveChanged_Trig_ZC = b_varargout_1;

  // End of Outputs for SubSystem: '<Root>/Check if Gains Have Changed'

  // MATLABSystem: '<S8>/SourceBlock'
  b_varargout_1_0 = Sub_PID_2028.getLatestMessage(&b_varargout_2);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S22>/Enable'

  if (b_varargout_1_0) {
    // SignalConversion generated from: '<S22>/In1'
    PID_B.In1_p = b_varargout_2;
  }

  // End of MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'

  // Outputs for Triggered SubSystem: '<Root>/Control System' incorporates:
  //   TriggerPort: '<S2>/Trigger'

  // MATLABSystem: '<S6>/SourceBlock'
  if (b_varargout_1 && (PID_PrevZCX.ControlSystem_Trig_ZCE != POS_ZCSIG)) {
    // MATLAB Function: '<S2>/Normalize Error' incorporates:
    //   MATLAB Function: '<Root>/MATLAB Function'
    //   SignalConversion generated from: '<S17>/ SFunction '
    //   SignalConversion generated from: '<S4>/ SFunction '

    PID_B.normalizedError[0] = PID_B.In1.pose.pose.position.x -
      PID_B.In1_c.position.x;
    PID_B.normalizedError[1] = PID_B.In1.pose.pose.position.y -
      PID_B.In1_c.position.y;
    PID_B.normalizedError[2] = PID_B.In1.pose.pose.position.z -
      PID_B.In1_c.position.z;
    PID_B.IfForceSurfaceGains = 3.3121686421112381E-170;
    PID_B.absxk = fabs(PID_B.In1.pose.pose.orientation.w);
    if (PID_B.absxk > 3.3121686421112381E-170) {
      PID_B.a = 1.0;
      PID_B.IfForceSurfaceGains = PID_B.absxk;
    } else {
      PID_B.t = PID_B.absxk / 3.3121686421112381E-170;
      PID_B.a = PID_B.t * PID_B.t;
    }

    PID_B.absxk = fabs(PID_B.In1.pose.pose.orientation.x);
    if (PID_B.absxk > PID_B.IfForceSurfaceGains) {
      PID_B.t = PID_B.IfForceSurfaceGains / PID_B.absxk;
      PID_B.a = PID_B.a * PID_B.t * PID_B.t + 1.0;
      PID_B.IfForceSurfaceGains = PID_B.absxk;
    } else {
      PID_B.t = PID_B.absxk / PID_B.IfForceSurfaceGains;
      PID_B.a += PID_B.t * PID_B.t;
    }

    PID_B.absxk = fabs(PID_B.In1.pose.pose.orientation.y);
    if (PID_B.absxk > PID_B.IfForceSurfaceGains) {
      PID_B.t = PID_B.IfForceSurfaceGains / PID_B.absxk;
      PID_B.a = PID_B.a * PID_B.t * PID_B.t + 1.0;
      PID_B.IfForceSurfaceGains = PID_B.absxk;
    } else {
      PID_B.t = PID_B.absxk / PID_B.IfForceSurfaceGains;
      PID_B.a += PID_B.t * PID_B.t;
    }

    PID_B.absxk = fabs(PID_B.In1.pose.pose.orientation.z);
    if (PID_B.absxk > PID_B.IfForceSurfaceGains) {
      PID_B.t = PID_B.IfForceSurfaceGains / PID_B.absxk;
      PID_B.a = PID_B.a * PID_B.t * PID_B.t + 1.0;
      PID_B.IfForceSurfaceGains = PID_B.absxk;
    } else {
      PID_B.t = PID_B.absxk / PID_B.IfForceSurfaceGains;
      PID_B.a += PID_B.t * PID_B.t;
    }

    PID_B.IfForceSurfaceGains *= sqrt(PID_B.a);
    PID_B.IfForceSurfaceGains *= PID_B.IfForceSurfaceGains;
    PID_B.W2B_p[0] = PID_B.In1.pose.pose.orientation.w /
      PID_B.IfForceSurfaceGains;
    PID_B.W2B_p[1] = -PID_B.In1.pose.pose.orientation.x /
      PID_B.IfForceSurfaceGains;
    PID_B.W2B_p[2] = -PID_B.In1.pose.pose.orientation.y /
      PID_B.IfForceSurfaceGains;
    PID_B.W2B_p[3] = -PID_B.In1.pose.pose.orientation.z /
      PID_B.IfForceSurfaceGains;
    PID_quatmultiply(b, PID_B.W2B_p, PID_B.W2B);
    PID_B.rtb_TmpSignalConversionAtSFun_c[0] = PID_B.In1_c.orientation.w;
    PID_B.rtb_TmpSignalConversionAtSFun_c[1] = -PID_B.In1_c.orientation.x;
    PID_B.rtb_TmpSignalConversionAtSFun_c[2] = -PID_B.In1_c.orientation.y;
    PID_B.rtb_TmpSignalConversionAtSFun_c[3] = -PID_B.In1_c.orientation.z;
    PID_B.W2B_p[0] = PID_B.W2B[0];
    PID_B.W2B_p[1] = -PID_B.W2B[1];
    PID_B.W2B_p[2] = -PID_B.W2B[2];
    PID_B.W2B_p[3] = -PID_B.W2B[3];
    PID_quatmultiply(PID_B.rtb_TmpSignalConversionAtSFun_c, PID_B.W2B_p,
                     PID_B.W2B);
    PID_B.axisError[0] = PID_B.W2B[1];
    PID_B.axisError[1] = PID_B.W2B[2];
    PID_B.axisError[2] = PID_B.W2B[3];
    if (PID_B.W2B[0] < 0.0) {
      PID_B.axisError[0] = -PID_B.W2B[1];
      PID_B.axisError[1] = -PID_B.W2B[2];
      PID_B.axisError[2] = -PID_B.W2B[3];
    }

    PID_B.normalizedError[3] = PID_B.axisError[0];
    PID_B.normalizedError[4] = PID_B.axisError[1];
    PID_B.normalizedError[5] = PID_B.axisError[2];

    // End of MATLAB Function: '<S2>/Normalize Error'

    // MATLAB Function: '<S2>/Manage Accumulated Error' incorporates:
    //   MATLABSystem: '<Root>/Current Time'
    //   Memory: '<S2>/Accumulated Error'
    //   Memory: '<S2>/Previous Time'

    if (PID_DW.PreviousTime_PreviousInput <= 0.0) {
      for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
        PID_B.Divide4[PID_B.i] = 0.0;
      }
    } else {
      PID_B.IfForceSurfaceGains = PID_B.Divide8 -
        PID_DW.PreviousTime_PreviousInput;
      if (PID_B.haveGainsChanged || PID_B.In1_p.data) {
        for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
          PID_B.Divide4[PID_B.i] = 0.0;
        }
      } else {
        for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
          PID_B.absxk = PID_B.normalizedError[PID_B.i];
          PID_B.Divide4[PID_B.i] = PID_B.absxk * PID_B.IfForceSurfaceGains +
            PID_DW.AccumulatedError_PreviousInput[PID_B.i];
          if (PID_B.absxk > PID_B.Divide) {
            PID_B.Divide4[PID_B.i] = 0.0;
          }

          if (rtIsNaN(PID_B.absxk)) {
            PID_B.Divide4[PID_B.i] = 0.0;
          }
        }
      }
    }

    // End of MATLAB Function: '<S2>/Manage Accumulated Error'

    // Product: '<S2>/P Control Negative'
    PID_B.IfSurfaceGains1[0] = PID_B.WorldGain_e;
    PID_B.IfSurfaceGains1[1] = PID_B.WorldGain_d;
    PID_B.IfSurfaceGains1[2] = PID_B.q_idx_2;
    PID_B.IfSurfaceGains1[3] = PID_B.q_idx_3;
    PID_B.IfSurfaceGains1[4] = PID_B.WorldGain_eo;
    PID_B.IfSurfaceGains1[5] = PID_B.WorldGain_pm;

    // Product: '<S2>/D Control Negative'
    PID_B.rtb_WorldGain_p3_c[0] = PID_B.WorldGain_p3;
    PID_B.rtb_WorldGain_p3_c[1] = PID_B.WorldGain_g2;
    PID_B.rtb_WorldGain_p3_c[2] = PID_B.WorldGain_l;
    PID_B.rtb_WorldGain_p3_c[3] = PID_B.WorldGain_ad;
    PID_B.rtb_WorldGain_p3_c[4] = PID_B.WorldGain_c;
    PID_B.rtb_WorldGain_p3_c[5] = PID_B.WorldGain_px;
    for (PID_B.i = 0; PID_B.i < 3; PID_B.i++) {
      // Product: '<S2>/D Control Negative' incorporates:
      //   Math: '<Root>/Body 2 World'
      //   Product: '<S13>/MatrixMultiply8'
      //   SignalConversion generated from: '<S13>/MatrixMultiply8'

      PID_B.rtb_Body2World_k[PID_B.i] = (PID_B.Body2World[PID_B.i + 3] *
        PID_B.In1.twist.twist.linear.y + PID_B.Body2World[PID_B.i] *
        PID_B.In1.twist.twist.linear.x) + PID_B.Body2World[PID_B.i + 6] *
        PID_B.In1.twist.twist.linear.z;
    }

    // Product: '<S2>/D Control Negative'
    PID_B.rtb_Body2World_k[3] = PID_B.In1.twist.twist.angular.x;
    PID_B.rtb_Body2World_k[4] = PID_B.In1.twist.twist.angular.y;
    PID_B.rtb_Body2World_k[5] = PID_B.In1.twist.twist.angular.z;
    for (PID_B.i = 0; PID_B.i < 6; PID_B.i++) {
      // Product: '<S2>/I Control Negative' incorporates:
      //   Product: '<S2>/P Control Negative'

      PID_B.WorldGain_e = PID_B.Divide4[PID_B.i];

      // Sum: '<S2>/Control Sum' incorporates:
      //   Gain: '<S2>/D Control'
      //   Gain: '<S2>/I Control'
      //   Gain: '<S2>/P Control'
      //   Product: '<S2>/D Control Negative'
      //   Product: '<S2>/I Control Negative'
      //   Product: '<S2>/P Control Negative'
      //   Switch: '<S9>/If Surface Gains'

      PID_B.ControlSum[PID_B.i] = (PID_B.IfSurfaceGains1[PID_B.i] *
        PID_B.normalizedError[PID_B.i] * PID_P.PControl_Gain +
        PID_B.rtb_WorldGain_p3_c[PID_B.i] * PID_B.rtb_Body2World_k[PID_B.i] *
        PID_P.DControl_Gain) + PID_B.IfSurfaceGains[PID_B.i] * PID_B.WorldGain_e
        * PID_P.IControl_Gain;

      // Update for Memory: '<S2>/Accumulated Error'
      PID_DW.AccumulatedError_PreviousInput[PID_B.i] = PID_B.WorldGain_e;
    }

    // Update for Memory: '<S2>/Previous Time' incorporates:
    //   MATLABSystem: '<Root>/Current Time'

    PID_DW.PreviousTime_PreviousInput = PID_B.Divide8;
  }

  PID_PrevZCX.ControlSystem_Trig_ZCE = b_varargout_1;

  // End of Outputs for SubSystem: '<Root>/Control System'

  // Gain: '<S3>/Gain'
  PID_B.Divide8 = PID_P.Gain_Gain * PID_B.WorldGain_g;

  // MinMax: '<S3>/Max' incorporates:
  //   Sum: '<S2>/Control Sum'

  if ((!(PID_B.Divide8 >= PID_B.ControlSum[0])) && (!rtIsNaN(PID_B.ControlSum[0])))
  {
    PID_B.Divide8 = PID_B.ControlSum[0];
  }

  // MinMax: '<S3>/Min'
  if ((PID_B.WorldGain_g <= PID_B.Divide8) || rtIsNaN(PID_B.Divide8)) {
    PID_B.Divide8 = PID_B.WorldGain_g;
  }

  // Gain: '<S3>/Gain'
  PID_B.WorldGain_g = PID_P.Gain_Gain * PID_B.WorldGain_m;

  // MinMax: '<S3>/Max' incorporates:
  //   Sum: '<S2>/Control Sum'

  if ((!(PID_B.WorldGain_g >= PID_B.ControlSum[1])) && (!rtIsNaN
       (PID_B.ControlSum[1]))) {
    PID_B.WorldGain_g = PID_B.ControlSum[1];
  }

  // MinMax: '<S3>/Min'
  if ((PID_B.WorldGain_m <= PID_B.WorldGain_g) || rtIsNaN(PID_B.WorldGain_g)) {
    PID_B.normalizedError[1] = PID_B.WorldGain_m;
  } else {
    PID_B.normalizedError[1] = PID_B.WorldGain_g;
  }

  // Gain: '<S3>/Gain'
  PID_B.WorldGain_g = PID_P.Gain_Gain * PID_B.WorldGain_p;

  // MinMax: '<S3>/Max' incorporates:
  //   Sum: '<S2>/Control Sum'

  if ((!(PID_B.WorldGain_g >= PID_B.ControlSum[2])) && (!rtIsNaN
       (PID_B.ControlSum[2]))) {
    PID_B.WorldGain_g = PID_B.ControlSum[2];
  }

  // MinMax: '<S3>/Min'
  if ((PID_B.WorldGain_p <= PID_B.WorldGain_g) || rtIsNaN(PID_B.WorldGain_g)) {
    PID_B.WorldGain_g = PID_B.WorldGain_p;
  }

  // Gain: '<S3>/Gain'
  PID_B.WorldGain_m = PID_P.Gain_Gain * PID_B.q_idx_0;

  // MinMax: '<S3>/Max' incorporates:
  //   Sum: '<S2>/Control Sum'

  if ((!(PID_B.WorldGain_m >= PID_B.ControlSum[3])) && (!rtIsNaN
       (PID_B.ControlSum[3]))) {
    PID_B.WorldGain_m = PID_B.ControlSum[3];
  }

  // Gain: '<S3>/Gain'
  PID_B.WorldGain_p = PID_P.Gain_Gain * PID_B.q_idx_1;

  // MinMax: '<S3>/Max' incorporates:
  //   Sum: '<S2>/Control Sum'

  if ((!(PID_B.WorldGain_p >= PID_B.ControlSum[4])) && (!rtIsNaN
       (PID_B.ControlSum[4]))) {
    PID_B.WorldGain_p = PID_B.ControlSum[4];
  }

  // Gain: '<S3>/Gain'
  PID_B.WorldGain_e = PID_P.Gain_Gain * PID_B.WorldGain_a;

  // MinMax: '<S3>/Max' incorporates:
  //   Sum: '<S2>/Control Sum'

  if ((!(PID_B.WorldGain_e >= PID_B.ControlSum[5])) && (!rtIsNaN
       (PID_B.ControlSum[5]))) {
    PID_B.WorldGain_e = PID_B.ControlSum[5];
  }

  // Product: '<S14>/MatrixMultiply8' incorporates:
  //   MinMax: '<S3>/Min'

  for (PID_B.i = 0; PID_B.i < 3; PID_B.i++) {
    PID_B.axisError[PID_B.i] = (PID_B.world2BodyRotm[PID_B.i + 3] *
      PID_B.normalizedError[1] + PID_B.world2BodyRotm[PID_B.i] * PID_B.Divide8)
      + PID_B.world2BodyRotm[PID_B.i + 6] * PID_B.WorldGain_g;
  }

  // End of Product: '<S14>/MatrixMultiply8'

  // Outputs for Triggered SubSystem: '<Root>/Publish Control' incorporates:
  //   TriggerPort: '<S5>/Trigger'

  // MATLABSystem: '<S6>/SourceBlock'
  if (b_varargout_1 && (PID_PrevZCX.PublishControl_Trig_ZCE != POS_ZCSIG)) {
    // BusAssignment: '<S5>/Bus Assignment'
    PID_B.BusAssignment.linear.x = PID_B.axisError[0];
    PID_B.BusAssignment.linear.y = PID_B.axisError[1];
    PID_B.BusAssignment.linear.z = PID_B.axisError[2];

    // MinMax: '<S3>/Min'
    if ((PID_B.q_idx_0 <= PID_B.WorldGain_m) || rtIsNaN(PID_B.WorldGain_m)) {
      // BusAssignment: '<S5>/Bus Assignment'
      PID_B.BusAssignment.angular.x = PID_B.q_idx_0;
    } else {
      // BusAssignment: '<S5>/Bus Assignment'
      PID_B.BusAssignment.angular.x = PID_B.WorldGain_m;
    }

    if ((PID_B.q_idx_1 <= PID_B.WorldGain_p) || rtIsNaN(PID_B.WorldGain_p)) {
      // BusAssignment: '<S5>/Bus Assignment'
      PID_B.BusAssignment.angular.y = PID_B.q_idx_1;
    } else {
      // BusAssignment: '<S5>/Bus Assignment'
      PID_B.BusAssignment.angular.y = PID_B.WorldGain_p;
    }

    if ((PID_B.WorldGain_a <= PID_B.WorldGain_e) || rtIsNaN(PID_B.WorldGain_e))
    {
      // BusAssignment: '<S5>/Bus Assignment'
      PID_B.BusAssignment.angular.z = PID_B.WorldGain_a;
    } else {
      // BusAssignment: '<S5>/Bus Assignment'
      PID_B.BusAssignment.angular.z = PID_B.WorldGain_e;
    }

    // MATLABSystem: '<S19>/SinkBlock'
    Pub_PID_53.publish(&PID_B.BusAssignment);
  }

  PID_PrevZCX.PublishControl_Trig_ZCE = b_varargout_1;

  // End of Outputs for SubSystem: '<Root>/Publish Control'

  // Update for Switch: '<S9>/If Force Default Gains' incorporates:
  //   Memory: '<S9>/Previous Gain State'

  PID_DW.PreviousGainState_PreviousInput = 0.0;
  rate_scheduler((&PID_M));
}

// Model initialize function
void PID::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    int32_T i;
    char_T prmName[11];
    static const char_T tmp[22] = { 'p', 'i', 'd', '_', 's', 'c', 'a', 'l', 'i',
      'n', 'g', '_', 'p', 'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's' };

    static const char_T tmp_0[15] = { 'p', 'i', 'd', '_', 'm', 'a', 'x', '_',
      'c', 'o', 'n', 't', 'r', 'o', 'l' };

    static const char_T tmp_1[22] = { 'p', 'i', 'd', '_', 's', 'u', 'r', 'f',
      'a', 'c', 'e', '_', 'g', 'a', 'i', 'n', '_', 'f', 'l', 'o', 'o', 'r' };

    static const char_T tmp_2[23] = { 'p', 'i', 'd', '_', 's', 'u', 'r', 'f',
      'a', 'c', 'e', '_', 'g', 'a', 'i', 'n', '_', 'b', 'u', 'f', 'f', 'e', 'r'
    };

    static const char_T tmp_3[19] = { 'p', 'i', 'd', '_', 'P', '_', 's', 'u',
      'r', 'f', 'a', 'c', 'e', '_', 'g', 'a', 'i', 'n', 's' };

    static const char_T tmp_4[10] = { 'p', 'i', 'd', '_', 'P', 'G', 'a', 'i',
      'n', 's' };

    static const char_T tmp_5[19] = { 'p', 'i', 'd', '_', 'D', '_', 's', 'u',
      'r', 'f', 'a', 'c', 'e', '_', 'g', 'a', 'i', 'n', 's' };

    static const char_T tmp_6[10] = { 'p', 'i', 'd', '_', 'D', 'G', 'a', 'i',
      'n', 's' };

    static const char_T tmp_7[19] = { 'p', 'i', 'd', '_', 'r', 'e', 's', 'e',
      't', '_', 't', 'h', 'r', 'e', 's', 'h', 'o', 'l', 'd' };

    static const char_T tmp_8[19] = { 'p', 'i', 'd', '_', 'I', '_', 's', 'u',
      'r', 'f', 'a', 'c', 'e', '_', 'g', 'a', 'i', 'n', 's' };

    static const char_T tmp_9[10] = { 'p', 'i', 'd', '_', 'I', 'G', 'a', 'i',
      'n', 's' };

    // Start for IdentityMatrix: '<Root>/IdentityMatrix'
    memcpy(&PID_B.IdentityMatrix[0], &PID_P.IdentityMatrix_IDMatrixData[0], 9U *
           sizeof(real_T));
    PID_PrevZCX.CheckifGainsHaveChanged_Trig_ZC = POS_ZCSIG;
    PID_PrevZCX.ControlSystem_Trig_ZCE = POS_ZCSIG;
    PID_PrevZCX.PublishControl_Trig_ZCE = POS_ZCSIG;

    // InitializeConditions for Switch: '<S9>/If Force Default Gains' incorporates:
    //   Memory: '<S9>/Previous Gain State'

    PID_DW.PreviousGainState_PreviousInput =
      PID_P.PreviousGainState_InitialCondit;

    // SystemInitialize for Enabled SubSystem: '<S6>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S20>/In1' incorporates:
    //   Outport: '<S20>/Out1'

    PID_B.In1 = PID_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S21>/In1' incorporates:
    //   Outport: '<S21>/Out1'

    PID_B.In1_c = PID_P.Out1_Y0_d;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'

    // SystemInitialize for Triggered SubSystem: '<Root>/Check if Gains Have Changed' 
    // SystemInitialize for Outport: '<S1>/haveGainsChanged'
    PID_B.haveGainsChanged = PID_P.haveGainsChanged_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/Check if Gains Have Changed' 

    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S22>/In1' incorporates:
    //   Outport: '<S22>/Out1'

    PID_B.In1_p = PID_P.Out1_Y0_i;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'

    // SystemInitialize for Triggered SubSystem: '<Root>/Control System'
    // InitializeConditions for Memory: '<S2>/Previous Time'
    PID_DW.PreviousTime_PreviousInput = PID_P.PreviousTime_InitialCondition;

    // SystemInitialize for Triggered SubSystem: '<Root>/Check if Gains Have Changed' 
    for (i = 0; i < 6; i++) {
      // InitializeConditions for Memory: '<S1>/Previous P'
      PID_DW.PreviousP_PreviousInput[i] = PID_P.PreviousP_InitialCondition;

      // InitializeConditions for Memory: '<S1>/Previous I'
      PID_DW.PreviousI_PreviousInput[i] = PID_P.PreviousI_InitialCondition;

      // InitializeConditions for Memory: '<S1>/Previous D'
      PID_DW.PreviousD_PreviousInput[i] = PID_P.PreviousD_InitialCondition;

      // InitializeConditions for Memory: '<S2>/Accumulated Error'
      PID_DW.AccumulatedError_PreviousInput[i] =
        PID_P.AccumulatedError_InitialConditi;

      // SystemInitialize for Sum: '<S2>/Control Sum' incorporates:
      //   Outport: '<S2>/Control'

      PID_B.ControlSum[i] = PID_P.Control_Y0;
    }

    // End of SystemInitialize for SubSystem: '<Root>/Check if Gains Have Changed' 
    // End of SystemInitialize for SubSystem: '<Root>/Control System'

    // SystemInitialize for Triggered SubSystem: '<Root>/Publish Control'
    // Start for MATLABSystem: '<S19>/SinkBlock'
    PID_DW.obj_ng.isInitialized = 0;
    PID_DW.obj_ng.matlabCodegenIsDeleted = false;
    PID_SystemCore_setup(&PID_DW.obj_ng);

    // End of SystemInitialize for SubSystem: '<Root>/Publish Control'

    // Start for MATLABSystem: '<Root>/Get Parameter1'
    PID_DW.obj_j.matlabCodegenIsDeleted = false;
    PID_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      PID_B.prmName_g[i] = tmp[i];
    }

    PID_B.prmName_g[22] = '\x00';
    ParamGet_PID_44.initParam(&PID_B.prmName_g[0]);
    ParamGet_PID_44.setInitialValue(1000000L);
    PID_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Get Parameter1'

    // Start for MATLABSystem: '<Root>/max_control'
    PID_DW.obj_bm.matlabCodegenIsDeleted = false;
    PID_DW.obj_bm.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      PID_B.prmName_n[i] = tmp_0[i];
    }

    PID_B.prmName_n[15] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_123.initParam(&PID_B.prmName_n[0]);
    ParamGet_PID_123.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_bm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/max_control'

    // Start for MATLABSystem: '<S6>/SourceBlock'
    PID_DW.obj_ex.isInitialized = 0;
    PID_DW.obj_ex.matlabCodegenIsDeleted = false;
    PID_SystemCore_setup_n(&PID_DW.obj_ex);

    // Start for MATLABSystem: '<Root>/surface gain floor'
    PID_DW.obj_c.matlabCodegenIsDeleted = false;
    PID_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      PID_B.prmName_g[i] = tmp_1[i];
    }

    PID_B.prmName_g[22] = '\x00';
    ParamGet_PID_240.initParam(&PID_B.prmName_g[0]);
    ParamGet_PID_240.setInitialValue(-750000L);
    PID_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/surface gain floor'

    // Start for MATLABSystem: '<Root>/surface gain buffer'
    PID_DW.obj_h.matlabCodegenIsDeleted = false;
    PID_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      PID_B.prmName[i] = tmp_2[i];
    }

    PID_B.prmName[23] = '\x00';
    ParamGet_PID_239.initParam(&PID_B.prmName[0]);
    ParamGet_PID_239.setInitialValue(250000L);
    PID_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/surface gain buffer'

    // Start for MATLABSystem: '<Root>/p surface'
    PID_DW.obj_b.matlabCodegenIsDeleted = false;
    PID_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      PID_B.prmName_m[i] = tmp_3[i];
    }

    PID_B.prmName_m[19] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_235.initParam(&PID_B.prmName_m[0]);
    ParamGet_PID_235.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/p surface'

    // Start for MATLABSystem: '<Root>/p Gains'
    PID_DW.obj_g.matlabCodegenIsDeleted = false;
    PID_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      prmName[i] = tmp_4[i];
    }

    prmName[10] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_48.initParam(&prmName[0]);
    ParamGet_PID_48.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/p Gains'

    // Start for MATLABSystem: '<Root>/d surface'
    PID_DW.obj_a.matlabCodegenIsDeleted = false;
    PID_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      PID_B.prmName_m[i] = tmp_5[i];
    }

    PID_B.prmName_m[19] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_233.initParam(&PID_B.prmName_m[0]);
    ParamGet_PID_233.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/d surface'

    // Start for MATLABSystem: '<Root>/d Gains'
    PID_DW.obj_e.matlabCodegenIsDeleted = false;
    PID_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      prmName[i] = tmp_6[i];
    }

    prmName[10] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_47.initParam(&prmName[0]);
    ParamGet_PID_47.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/d Gains'

    // Start for MATLABSystem: '<S7>/SourceBlock'
    PID_DW.obj_k.isInitialized = 0;
    PID_DW.obj_k.matlabCodegenIsDeleted = false;
    PID_SystemCore_setup_nt(&PID_DW.obj_k);

    // Start for MATLABSystem: '<Root>/Get Parameter'
    PID_DW.obj_d.matlabCodegenIsDeleted = false;
    PID_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      PID_B.prmName_m[i] = tmp_7[i];
    }

    PID_B.prmName_m[19] = '\x00';
    ParamGet_PID_43.initParam(&PID_B.prmName_m[0]);
    ParamGet_PID_43.setInitialValue(100000L);
    PID_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Get Parameter'

    // Start for MATLABSystem: '<Root>/Current Time'
    PID_DW.obj.matlabCodegenIsDeleted = false;
    PID_DW.obj.isInitialized = 1;
    PID_DW.obj.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/i surface'
    PID_DW.obj_bv.matlabCodegenIsDeleted = false;
    PID_DW.obj_bv.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      PID_B.prmName_m[i] = tmp_8[i];
    }

    PID_B.prmName_m[19] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_234.initParam(&PID_B.prmName_m[0]);
    ParamGet_PID_234.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_bv.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/i surface'

    // Start for MATLABSystem: '<Root>/i Gains'
    PID_DW.obj_n.matlabCodegenIsDeleted = false;
    PID_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      prmName[i] = tmp_9[i];
    }

    prmName[10] = '\x00';
    for (i = 0; i < 6; i++) {
      PID_B.varargin_1[i] = 0L;
    }

    ParamGet_PID_46.initParam(&prmName[0]);
    ParamGet_PID_46.setInitialValue(&PID_B.varargin_1[0], 6U);
    PID_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/i Gains'

    // Start for MATLABSystem: '<S8>/SourceBlock'
    PID_DW.obj_av.isInitialized = 0;
    PID_DW.obj_av.matlabCodegenIsDeleted = false;
    PID_SystemCore_setup_ntv(&PID_DW.obj_av);
  }
}

// Model terminate function
void PID::terminate()
{
  // Terminate for MATLABSystem: '<Root>/Get Parameter1'
  if (!PID_DW.obj_j.matlabCodegenIsDeleted) {
    PID_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Get Parameter1'

  // Terminate for MATLABSystem: '<Root>/max_control'
  if (!PID_DW.obj_bm.matlabCodegenIsDeleted) {
    PID_DW.obj_bm.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/max_control'

  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  if (!PID_DW.obj_ex.matlabCodegenIsDeleted) {
    PID_DW.obj_ex.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S6>/SourceBlock'

  // Terminate for MATLABSystem: '<Root>/surface gain floor'
  if (!PID_DW.obj_c.matlabCodegenIsDeleted) {
    PID_DW.obj_c.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/surface gain floor'

  // Terminate for MATLABSystem: '<Root>/surface gain buffer'
  if (!PID_DW.obj_h.matlabCodegenIsDeleted) {
    PID_DW.obj_h.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/surface gain buffer'

  // Terminate for MATLABSystem: '<Root>/p surface'
  if (!PID_DW.obj_b.matlabCodegenIsDeleted) {
    PID_DW.obj_b.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/p surface'

  // Terminate for MATLABSystem: '<Root>/p Gains'
  if (!PID_DW.obj_g.matlabCodegenIsDeleted) {
    PID_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/p Gains'

  // Terminate for MATLABSystem: '<Root>/d surface'
  if (!PID_DW.obj_a.matlabCodegenIsDeleted) {
    PID_DW.obj_a.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/d surface'

  // Terminate for MATLABSystem: '<Root>/d Gains'
  if (!PID_DW.obj_e.matlabCodegenIsDeleted) {
    PID_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/d Gains'

  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  if (!PID_DW.obj_k.matlabCodegenIsDeleted) {
    PID_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S7>/SourceBlock'

  // Terminate for MATLABSystem: '<Root>/Get Parameter'
  if (!PID_DW.obj_d.matlabCodegenIsDeleted) {
    PID_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Get Parameter'

  // Terminate for MATLABSystem: '<Root>/Current Time'
  if (!PID_DW.obj.matlabCodegenIsDeleted) {
    PID_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Current Time'

  // Terminate for MATLABSystem: '<Root>/i surface'
  if (!PID_DW.obj_bv.matlabCodegenIsDeleted) {
    PID_DW.obj_bv.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/i surface'

  // Terminate for MATLABSystem: '<Root>/i Gains'
  if (!PID_DW.obj_n.matlabCodegenIsDeleted) {
    PID_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/i Gains'

  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  if (!PID_DW.obj_av.matlabCodegenIsDeleted) {
    PID_DW.obj_av.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S8>/SourceBlock'

  // Terminate for Triggered SubSystem: '<Root>/Publish Control'
  // Terminate for MATLABSystem: '<S19>/SinkBlock'
  if (!PID_DW.obj_ng.matlabCodegenIsDeleted) {
    PID_DW.obj_ng.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S19>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish Control'
}

// Constructor
PID::PID() :
  PID_B(),
  PID_DW(),
  PID_PrevZCX(),
  PID_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
PID::~PID()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_PID_T * PID::getRTM()
{
  return (&PID_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
