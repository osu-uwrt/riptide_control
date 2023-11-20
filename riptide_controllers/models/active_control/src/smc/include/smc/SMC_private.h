//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SMC_private.h
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
#ifndef RTW_HEADER_SMC_private_h_
#define RTW_HEADER_SMC_private_h_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "SMC_types.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif
#endif                                 // RTW_HEADER_SMC_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
