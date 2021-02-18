//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: IMU_Fusion_data.cpp
//
// Code generated for Simulink model 'IMU_Fusion'.
//
// Model version                  : 1.5
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Thu Feb 18 02:03:51 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. ROM efficiency
//    3. RAM efficiency
// Validation result: Not run
//
#include "IMU_Fusion.h"
#include "IMU_Fusion_private.h"

// Block parameters (default storage)
IMU_FusionModelClass::P_IMU_Fusion_T IMU_FusionModelClass::IMU_Fusion_P = {
  // Expression: accelNoise
  //  Referenced by: '<Root>/AHRS'

  0.0001924722,

  // Expression: gyroNoise
  //  Referenced by: '<Root>/AHRS'

  9.1385E-5,

  // Expression: magNoise
  //  Referenced by: '<Root>/AHRS'

  0.1,

  // Expression: gyroDriftNoise
  //  Referenced by: '<Root>/AHRS'

  3.0462E-13,

  // Expression: linAccelNoise
  //  Referenced by: '<Root>/AHRS'

  0.0096236100000000012,

  // Expression: magDisturbanceNoise
  //  Referenced by: '<Root>/AHRS'

  0.5,

  // Expression: linDecayFactor
  //  Referenced by: '<Root>/AHRS'

  0.5,

  // Expression: magDecayFactor
  //  Referenced by: '<Root>/AHRS'

  0.5,

  // Expression: magFieldStrength
  //  Referenced by: '<Root>/AHRS'

  50.0,

  // Expression: 1
  //  Referenced by: '<S6>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S7>/Constant'

  1.0,

  // Expression: 10
  //  Referenced by: '<Root>/Constant'

  10.0
};

//
// File trailer for generated code.
//
// [EOF]
//
