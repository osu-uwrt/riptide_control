//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID_data.cpp
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

// Block parameters (default storage)
P_PID_T PID::PID_P = {
  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S20>/Out1'

  {
    {
      {
        0,                             // sec
        0U                             // nanosec
      },                               // stamp

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // frame_id

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      }                                // frame_id_SL_Info
    },                                 // header

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // child_frame_id

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // child_frame_id_SL_Info

    {
      {
        {
          0.0,                         // x
          0.0,                         // y
          0.0                          // z
        },                             // position

        {
          0.0,                         // x
          0.0,                         // y
          0.0,                         // z
          0.0                          // w
        }                              // orientation
      },                               // pose

      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      // covariance
    },                                 // pose

    {
      {
        {
          0.0,                         // x
          0.0,                         // y
          0.0                          // z
        },                             // linear

        {
          0.0,                         // x
          0.0,                         // y
          0.0                          // z
        }                              // angular
      },                               // twist

      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      // covariance
    }                                  // twist
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S6>/Constant'

  {
    {
      {
        0,                             // sec
        0U                             // nanosec
      },                               // stamp

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // frame_id

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      }                                // frame_id_SL_Info
    },                                 // header

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // child_frame_id

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // child_frame_id_SL_Info

    {
      {
        {
          0.0,                         // x
          0.0,                         // y
          0.0                          // z
        },                             // position

        {
          0.0,                         // x
          0.0,                         // y
          0.0,                         // z
          0.0                          // w
        }                              // orientation
      },                               // pose

      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      // covariance
    },                                 // pose

    {
      {
        {
          0.0,                         // x
          0.0,                         // y
          0.0                          // z
        },                             // linear

        {
          0.0,                         // x
          0.0,                         // y
          0.0                          // z
        }                              // angular
      },                               // twist

      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      // covariance
    }                                  // twist
  },

  // Computed Parameter: Out1_Y0_d
  //  Referenced by: '<S21>/Out1'

  {
    {
      0.0,                             // x
      0.0,                             // y
      0.0                              // z
    },                                 // position

    {
      0.0,                             // x
      0.0,                             // y
      0.0,                             // z
      0.0                              // w
    }                                  // orientation
  },

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S7>/Constant'

  {
    {
      0.0,                             // x
      0.0,                             // y
      0.0                              // z
    },                                 // position

    {
      0.0,                             // x
      0.0,                             // y
      0.0,                             // z
      0.0                              // w
    }                                  // orientation
  },

  // Computed Parameter: Constant_Value_m
  //  Referenced by: '<S18>/Constant'

  {
    {
      0.0,                             // x
      0.0,                             // y
      0.0                              // z
    },                                 // linear

    {
      0.0,                             // x
      0.0,                             // y
      0.0                              // z
    }                                  // angular
  },

  // Computed Parameter: Out1_Y0_i
  //  Referenced by: '<S22>/Out1'

  {
    false                              // data
  },

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S8>/Constant'

  {
    false                              // data
  },

  // Expression: 0
  //  Referenced by: '<S1>/Previous P'

  0.0,

  // Expression: 0
  //  Referenced by: '<S1>/Previous I'

  0.0,

  // Expression: 0
  //  Referenced by: '<S1>/Previous D'

  0.0,

  // Computed Parameter: Control_Y0
  //  Referenced by: '<S2>/Control'

  0.0,

  // Expression: 0
  //  Referenced by: '<S2>/Accumulated Error'

  0.0,

  // Expression: -1
  //  Referenced by: '<S2>/Previous Time'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S2>/I Control'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S2>/P Control'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S2>/D Control'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S9>/Constant'

  0.0,

  // Expression: 1
  //  Referenced by: '<S9>/Constant1'

  1.0,

  // Computed Parameter: IdentityMatrix_IDMatrixData
  //  Referenced by: '<Root>/IdentityMatrix'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: 0
  //  Referenced by: '<S9>/Previous Gain State'

  0.0,

  // Expression: 0
  //  Referenced by: '<S9>/If Force Default Gains'

  0.0,

  // Expression: 0
  //  Referenced by: '<S9>/If Force Surface Gains'

  0.0,

  // Expression: 0
  //  Referenced by: '<S9>/If Surface Gains2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S9>/If Surface Gains1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S9>/If Surface Gains'

  0.0,

  // Expression: -1
  //  Referenced by: '<S3>/Gain'

  -1.0,

  // Computed Parameter: haveGainsChanged_Y0
  //  Referenced by: '<S1>/haveGainsChanged'

  false,

  // Start of '<Root>/Tranfsform Gains3'
  {
    // Expression: [1;0;0]
    //  Referenced by: '<S12>/World Principal X'

    { 1.0, 0.0, 0.0 },

    // Expression: -1/2
    //  Referenced by: '<S35>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S35>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S35>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S35>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S35>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S35>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S35>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S35>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S35>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S36>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S36>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S36>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S36>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S36>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S36>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S36>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S36>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S36>/Stop Infinite Tanget 2'

    0.0,

    // Expression: [0;1;0]
    //  Referenced by: '<S12>/World Principal Y'

    { 0.0, 1.0, 0.0 },

    // Expression: -1/2
    //  Referenced by: '<S37>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S37>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S37>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S37>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S37>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S37>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S37>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S37>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S37>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S38>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S38>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S38>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S38>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S38>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S38>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S38>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S38>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S38>/Stop Infinite Tanget 2'

    0.0,

    // Expression: [0;0;1]
    //  Referenced by: '<S12>/World Principal Z'

    { 0.0, 0.0, 1.0 },

    // Expression: -1/2
    //  Referenced by: '<S39>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S39>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S39>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S39>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S39>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S39>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S39>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S39>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S39>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S40>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S40>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S40>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S40>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S40>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S40>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S40>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S40>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S40>/Stop Infinite Tanget 2'

    0.0
  }
  ,

  // End of '<Root>/Tranfsform Gains3'

  // Start of '<Root>/Tranfsform Gains2'
  {
    // Expression: [1;0;0]
    //  Referenced by: '<S11>/World Principal X'

    { 1.0, 0.0, 0.0 },

    // Expression: -1/2
    //  Referenced by: '<S29>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S29>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S29>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S29>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S29>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S29>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S29>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S29>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S29>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S30>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S30>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S30>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S30>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S30>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S30>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S30>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S30>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S30>/Stop Infinite Tanget 2'

    0.0,

    // Expression: [0;1;0]
    //  Referenced by: '<S11>/World Principal Y'

    { 0.0, 1.0, 0.0 },

    // Expression: -1/2
    //  Referenced by: '<S31>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S31>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S31>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S31>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S31>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S31>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S31>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S31>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S31>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S32>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S32>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S32>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S32>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S32>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S32>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S32>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S32>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S32>/Stop Infinite Tanget 2'

    0.0,

    // Expression: [0;0;1]
    //  Referenced by: '<S11>/World Principal Z'

    { 0.0, 0.0, 1.0 },

    // Expression: -1/2
    //  Referenced by: '<S33>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S33>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S33>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S33>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S33>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S33>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S33>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S33>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S33>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S34>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S34>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S34>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S34>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S34>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S34>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S34>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S34>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S34>/Stop Infinite Tanget 2'

    0.0
  }
  ,

  // End of '<Root>/Tranfsform Gains2'

  // Start of '<Root>/Tranfsform Gains1'
  {
    // Expression: [1;0;0]
    //  Referenced by: '<S10>/World Principal X'

    { 1.0, 0.0, 0.0 },

    // Expression: -1/2
    //  Referenced by: '<S23>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S23>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S23>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S23>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S23>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S23>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S23>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S23>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S23>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S24>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S24>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S24>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S24>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S24>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S24>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S24>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S24>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S24>/Stop Infinite Tanget 2'

    0.0,

    // Expression: [0;1;0]
    //  Referenced by: '<S10>/World Principal Y'

    { 0.0, 1.0, 0.0 },

    // Expression: -1/2
    //  Referenced by: '<S25>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S25>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S25>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S25>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S25>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S25>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S25>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S25>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S25>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S26>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S26>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S26>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S26>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S26>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S26>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S26>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S26>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S26>/Stop Infinite Tanget 2'

    0.0,

    // Expression: [0;0;1]
    //  Referenced by: '<S10>/World Principal Z'

    { 0.0, 0.0, 1.0 },

    // Expression: -1/2
    //  Referenced by: '<S27>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S27>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S27>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S27>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S27>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S27>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S27>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S27>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S27>/Stop Infinite Tanget 2'

    0.0,

    // Expression: -1/2
    //  Referenced by: '<S28>/Constant1'

    -0.5,

    // Expression: 1
    //  Referenced by: '<S28>/Constant2'

    1.0,

    // Expression: 999999
    //  Referenced by: '<S28>/Infinity'

    999999.0,

    // Expression: 0
    //  Referenced by: '<S28>/Non Zero Gains'

    0.0,

    // Expression: 1
    //  Referenced by: '<S28>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S28>/Non Zero Gains 1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S28>/Stop Infinite Tangent'

    0.0,

    // Expression: 0
    //  Referenced by: '<S28>/Non Zero Gains 2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S28>/Stop Infinite Tanget 2'

    0.0
  }
  // End of '<Root>/Tranfsform Gains1'
};

//
// File trailer for generated code.
//
// [EOF]
//
