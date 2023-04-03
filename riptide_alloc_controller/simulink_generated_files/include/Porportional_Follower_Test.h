/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Porportional_Follower_Test.h
 *
 * Code generated for Simulink model 'Porportional_Follower_Test'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Mon Mar 27 19:02:21 2023
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Porportional_Follower_Test_h_
#define RTW_HEADER_Porportional_Follower_Test_h_
#ifndef Porportional_Follower_Test_COMMON_INCLUDES_
#define Porportional_Follower_Test_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                         /* Porportional_Follower_Test_COMMON_INCLUDES_ */

#include "Porportional_Follower_Test_types.h"

/* Macros for accessing real-time model data structure */
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

/* Block signals (default storage) */
typedef struct {
  real_T Clock;                        /* '<S2>/Clock' */
  real_T Add;                          /* '<S1>/Add' */
  real_T Add1;                         /* '<S1>/Add1' */
} B_Porportional_Follower_Test_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T PreviousSignalValue_PreviousInp;/* '<S1>/Previous Signal Value' */
  real_T Memory2_PreviousInput;        /* '<S2>/Memory2' */
  real_T PreviousVelocity_PreviousInput;/* '<S1>/Previous Velocity' */
} DW_Porportional_Follower_Test_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Signal;                       /* '<Root>/Signal' */
} ExtU_Porportional_Follower_Te_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Follower;                     /* '<Root>/Follower' */
} ExtY_Porportional_Follower_Te_T;

/* Real-time Model Data Structure */
struct tag_RTM_Porportional_Follower_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_Porportional_Follower_Test_T Porportional_Follower_Test_B;

/* Block states (default storage) */
extern DW_Porportional_Follower_Test_T Porportional_Follower_Test_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Porportional_Follower_Te_T Porportional_Follower_Test_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Porportional_Follower_Te_T Porportional_Follower_Test_Y;

/* Model entry point functions */
extern void Porportional_Follower_Test_initialize(void);
extern void Porportional_Follower_Test_step(void);
extern void Porportional_Follower_Test_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Porportional_Followe_T *const Porportional_Follower_Test_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('TestController/Porportional_Follower_Test')    - opens subsystem TestController/Porportional_Follower_Test
 * hilite_system('TestController/Porportional_Follower_Test/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'TestController'
 * '<S1>'   : 'TestController/Porportional_Follower_Test'
 * '<S2>'   : 'TestController/Porportional_Follower_Test/Delta Time'
 */
#endif                            /* RTW_HEADER_Porportional_Follower_Test_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
