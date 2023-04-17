/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FaultControllerModel.h
 *
 * Code generated for Simulink model 'FaultControllerModel'.
 *
 * Model version                  : 1.31
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sun Apr 16 15:36:33 2023
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_FaultControllerModel_h_
#define RTW_HEADER_FaultControllerModel_h_
#ifndef FaultControllerModel_COMMON_INCLUDES_
#define FaultControllerModel_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                               /* FaultControllerModel_COMMON_INCLUDES_ */

#include "FaultControllerModel_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Reciprocal_DWORK4[36];        /* '<S2>/Reciprocal' */
} DW_FaultControllerModel_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T RobotMass;                    /* '<Root>/Robot Mass' */
  real_T RobotVolume;                  /* '<Root>/Robot Volume' */
  real_T RobotCenterofMass[3];         /* '<Root>/Robot Center of Mass' */
  real_T RobotCenterofBuoyancy[3];     /* '<Root>/Robot Center of Buoyancy' */
  real_T LinearPGains[9];              /* '<Root>/Linear P Gains' */
  real_T AngularPGains[9];             /* '<Root>/Angular P Gains' */
  real_T LinearVelocityGains[9];       /* '<Root>/Linear Velocity Gains' */
  real_T AngularVelocityGains[9];      /* '<Root>/Angular Velocity Gains' */
  real_T ThrusterConfiguration[48];    /* '<Root>/Thruster Configuration' */
  boolean_T ControllerType;            /* '<Root>/Controller Type' */
  real_T RobotMomentsofInertia[3];     /* '<Root>/Robot Moments of Inertia' */
  real_T RobotLinearPosition[3];       /* '<Root>/Robot Linear Position' */
  real_T RobotOrientation[4];          /* '<Root>/Robot Orientation' */
  real_T RobotLinearVelocity[3];       /* '<Root>/Robot Linear Velocity' */
  real_T RobotAngularVelocity[3];      /* '<Root>/Robot Angular Velocity' */
  real_T ReferencelinearPosition[3];   /* '<Root>/Reference linear Position' */
  real_T Referenceorientation[4];      /* '<Root>/Reference orientation' */
  real_T ReferenceLinearVelocity;      /* '<Root>/Reference Linear Velocity' */
  real_T ReferenceAngularVelocity;     /* '<Root>/Reference Angular Velocity' */
  real_T LinearDragCoefficients[6];    /* '<Root>/Linear Drag Coefficients' */
  real_T QuadraticDragCoefficients[6];/* '<Root>/Quadratic Drag Coefficients' */
  real_T ThrusterDirections;           /* '<Root>/Thruster Directions' */
  real_T MiscThrusterforcetoDShotValues[11];
                              /* '<Root>/Misc Thruster force to DShot Values' */
} ExtU_FaultControllerModel_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  int16_T DshotCommands[8];            /* '<Root>/Dshot Commands' */
  real_T ThrusterForces[8];            /* '<Root>/Thruster Forces' */
  real_T NetForce[6];                  /* '<Root>/Net Force' */
} ExtY_FaultControllerModel_T;

/* Real-time Model Data Structure */
struct tag_RTM_FaultControllerModel_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_FaultControllerModel_T FaultControllerModel_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_FaultControllerModel_T FaultControllerModel_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_FaultControllerModel_T FaultControllerModel_Y;

/* Model entry point functions */
extern void FaultControllerModel_initialize(void);
extern void FaultControllerModel_step(void);
extern void FaultControllerModel_terminate(void);

/* Real-time Model object */
extern RT_MODEL_FaultControllerModel_T *const FaultControllerModel_M;

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
 * hilite_system('FaultControllerTest/FaultControllerModel')    - opens subsystem FaultControllerTest/FaultControllerModel
 * hilite_system('FaultControllerTest/FaultControllerModel/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'FaultControllerTest'
 * '<S1>'   : 'FaultControllerTest/FaultControllerModel'
 * '<S2>'   : 'FaultControllerTest/FaultControllerModel/Allocate Thrusters'
 * '<S3>'   : 'FaultControllerTest/FaultControllerModel/Calculate Error'
 * '<S4>'   : 'FaultControllerTest/FaultControllerModel/Calculate Net Force'
 * '<S5>'   : 'FaultControllerTest/FaultControllerModel/Convert to DShot'
 * '<S6>'   : 'FaultControllerTest/FaultControllerModel/Allocate Thrusters/MATLAB Function'
 * '<S7>'   : 'FaultControllerTest/FaultControllerModel/Allocate Thrusters/Take inverse'
 * '<S8>'   : 'FaultControllerTest/FaultControllerModel/Calculate Error/MATLAB Function'
 * '<S9>'   : 'FaultControllerTest/FaultControllerModel/Calculate Net Force/Calculate Restoring Forces'
 * '<S10>'  : 'FaultControllerTest/FaultControllerModel/Calculate Net Force/Drag'
 * '<S11>'  : 'FaultControllerTest/FaultControllerModel/Calculate Net Force/Calculate Restoring Forces/Cross Product'
 * '<S12>'  : 'FaultControllerTest/FaultControllerModel/Calculate Net Force/Calculate Restoring Forces/Cross Product1'
 * '<S13>'  : 'FaultControllerTest/FaultControllerModel/Calculate Net Force/Calculate Restoring Forces/MATLAB Function'
 * '<S14>'  : 'FaultControllerTest/FaultControllerModel/Convert to DShot/Convert to DShot'
 * '<S15>'  : 'FaultControllerTest/FaultControllerModel/Convert to DShot/Create 8 row vector of the maximum thruster forces'
 */
#endif                                 /* RTW_HEADER_FaultControllerModel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
