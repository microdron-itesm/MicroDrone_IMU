//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: IMU_Fusion_types.h
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
#ifndef RTW_HEADER_IMU_Fusion_types_h_
#define RTW_HEADER_IMU_Fusion_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef struct_tag_v7m7NEstuUdktFlQqRQnjB
#define struct_tag_v7m7NEstuUdktFlQqRQnjB

struct tag_v7m7NEstuUdktFlQqRQnjB
{
  real_T a;
  real_T b;
  real_T c;
  real_T d;
};

#endif                                 //struct_tag_v7m7NEstuUdktFlQqRQnjB

#ifndef typedef_d_quaternion_IMU_Fusion_T
#define typedef_d_quaternion_IMU_Fusion_T

typedef tag_v7m7NEstuUdktFlQqRQnjB d_quaternion_IMU_Fusion_T;

#endif                                 //typedef_d_quaternion_IMU_Fusion_T

#ifndef struct_tag_yioZfVzJFClp3vmW2NrXQC
#define struct_tag_yioZfVzJFClp3vmW2NrXQC

struct tag_yioZfVzJFClp3vmW2NrXQC
{
  int32_T __dummy;
};

#endif                                 //struct_tag_yioZfVzJFClp3vmW2NrXQC

#ifndef typedef_c_fusion_internal_frames_NED__T
#define typedef_c_fusion_internal_frames_NED__T

typedef tag_yioZfVzJFClp3vmW2NrXQC c_fusion_internal_frames_NED__T;

#endif                                 //typedef_c_fusion_internal_frames_NED__T

#ifndef struct_tag_PMfBDzoakfdM9QAdfx2o6D
#define struct_tag_PMfBDzoakfdM9QAdfx2o6D

struct tag_PMfBDzoakfdM9QAdfx2o6D
{
  uint32_T f1[8];
};

#endif                                 //struct_tag_PMfBDzoakfdM9QAdfx2o6D

#ifndef typedef_cell_wrap_IMU_Fusion_T
#define typedef_cell_wrap_IMU_Fusion_T

typedef tag_PMfBDzoakfdM9QAdfx2o6D cell_wrap_IMU_Fusion_T;

#endif                                 //typedef_cell_wrap_IMU_Fusion_T

#ifndef struct_tag_wd2UHN16q7govLGCoSP4MH
#define struct_tag_wd2UHN16q7govLGCoSP4MH

struct tag_wd2UHN16q7govLGCoSP4MH
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_IMU_Fusion_T inputVarSize[3];
  real_T AccelerometerNoise;
  real_T GyroscopeNoise;
  real_T GyroscopeDriftNoise;
  real_T LinearAccelerationNoise;
  real_T LinearAccelerationDecayFactor;
  real_T pQw[144];
  real_T pQv[36];
  d_quaternion_IMU_Fusion_T pOrientPost;
  d_quaternion_IMU_Fusion_T pOrientPrior;
  boolean_T pFirstTime;
  c_fusion_internal_frames_NED__T *pRefSys;
  real_T pSensorPeriod;
  real_T pKalmanPeriod;
  real_T pGyroOffset[3];
  real_T pLinAccelPrior[3];
  real_T pLinAccelPost[3];
  real_T pInputPrototype[3];
  real_T MagnetometerNoise;
  real_T MagneticDisturbanceNoise;
  real_T MagneticDisturbanceDecayFactor;
  real_T ExpectedMagneticFieldStrength;
  real_T pMagVec[3];
  c_fusion_internal_frames_NED__T _pobj0;
};

#endif                                 //struct_tag_wd2UHN16q7govLGCoSP4MH

#ifndef typedef_fusion_simulink_ahrsfilter_IM_T
#define typedef_fusion_simulink_ahrsfilter_IM_T

typedef tag_wd2UHN16q7govLGCoSP4MH fusion_simulink_ahrsfilter_IM_T;

#endif                                 //typedef_fusion_simulink_ahrsfilter_IM_T
#endif                                 // RTW_HEADER_IMU_Fusion_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
