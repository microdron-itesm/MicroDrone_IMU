//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: IMU_Fusion.h
//
// Code generated for Simulink model 'IMU_Fusion'.
//
// Model version                  : 1.3
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Thu Feb 18 00:20:09 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_IMU_Fusion_h_
#define RTW_HEADER_IMU_Fusion_h_
#include <cmath>
#include <cstring>
#include "rtwtypes.h"
#include "IMU_Fusion_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model IMU_Fusion
class IMU_FusionModelClass {
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  typedef struct {
    fusion_simulink_ahrsfilter_IM_T obj;// '<Root>/AHRS'
  } DW_IMU_Fusion_T;

  // External inputs (root inport signals with default storage)
  typedef struct {
    real_T Accel_Raw[3];               // '<Root>/Accel_Raw'
    real_T Gyro_Raw[3];                // '<Root>/Gyro_Raw'
    real_T Mag_Raw[3];                 // '<Root>/Mag_Raw'
  } ExtU_IMU_Fusion_T;

  // External outputs (root outports fed by signals with default storage)
  typedef struct {
    real_T Quat[4];                    // '<Root>/Quat'
    real_T Euler[3];                   // '<Root>/Euler'
    real_T AngVel[3];                  // '<Root>/AngVel'
  } ExtY_IMU_Fusion_T;

  // Parameters (default storage)
  struct P_IMU_Fusion_T {
    real_T AHRS_AccelerometerNoise;    // Expression: accelNoise
                                          //  Referenced by: '<Root>/AHRS'

    real_T AHRS_GyroscopeNoise;        // Expression: gyroNoise
                                          //  Referenced by: '<Root>/AHRS'

    real_T AHRS_MagnetometerNoise;     // Expression: magNoise
                                          //  Referenced by: '<Root>/AHRS'

    real_T AHRS_GyroscopeDriftNoise;   // Expression: gyroDriftNoise
                                          //  Referenced by: '<Root>/AHRS'

    real_T AHRS_LinearAccelerationNoise;// Expression: linAccelNoise
                                           //  Referenced by: '<Root>/AHRS'

    real_T AHRS_MagneticDisturbanceNoise;// Expression: magDisturbanceNoise
                                            //  Referenced by: '<Root>/AHRS'

    real_T AHRS_LinearAccelerationDecayFac;// Expression: linDecayFactor
                                              //  Referenced by: '<Root>/AHRS'

    real_T AHRS_MagneticDisturbanceDecayFa;// Expression: magDecayFactor
                                              //  Referenced by: '<Root>/AHRS'

    real_T AHRS_ExpectedMagneticFieldStren;// Expression: magFieldStrength
                                              //  Referenced by: '<Root>/AHRS'

    real_T Constant_Value;             // Expression: 1
                                          //  Referenced by: '<S6>/Constant'

    real_T Constant_Value_l;           // Expression: 1
                                          //  Referenced by: '<S7>/Constant'

    real_T Constant_Value_e;           // Expression: 10
                                          //  Referenced by: '<Root>/Constant'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_IMU_Fusion_T {
    const char_T * volatile errorStatus;
  };

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  IMU_FusionModelClass();

  // Destructor
  ~IMU_FusionModelClass();

  // Root-level structure-based inputs set method

  // Root inports set method
  void setExternalInputs(const ExtU_IMU_Fusion_T* pExtU_IMU_Fusion_T)
  {
    IMU_Fusion_U = *pExtU_IMU_Fusion_T;
  }

  // Root-level structure-based outputs get method

  // Root outports get method
  const IMU_FusionModelClass::ExtY_IMU_Fusion_T & getExternalOutputs() const
  {
    return IMU_Fusion_Y;
  }

  // Real-Time Model get method
  IMU_FusionModelClass::RT_MODEL_IMU_Fusion_T * getRTM();

  // private data and function members
 private:
  // Tunable parameters
  static P_IMU_Fusion_T IMU_Fusion_P;

  // Block states
  DW_IMU_Fusion_T IMU_Fusion_DW;

  // External inputs
  ExtU_IMU_Fusion_T IMU_Fusion_U;

  // External outputs
  ExtY_IMU_Fusion_T IMU_Fusion_Y;

  // Real-Time Model
  RT_MODEL_IMU_Fusion_T IMU_Fusion_M;

  // private member function(s) for subsystem '<Root>'
  void IMUFusionCommon_computeAngularV(const real_T gfast[3], const real_T
    offset[3], real_T av[3]);
  void IMU_Fusion_NED_ecompass(const real_T a[3], const real_T m[3], real_T R[9]);
  void IMU_Fusio_quaternion_quaternion(const real_T varargin_1[9], real_T *obj_a,
    real_T *obj_b, real_T *obj_c, real_T *obj_d);
  void IMU_Fus_quaternion_quaternion_o(const real_T varargin_1[3], real_T *obj_a,
    real_T *obj_b, real_T *obj_c, real_T *obj_d);
  void IMUFusionCommon_predictOrientat(const fusion_simulink_ahrsfilter_IM_T
    *obj, const real_T gfast[3], const real_T offset[3], real_T qorient_a,
    real_T qorient_b, real_T qorient_c, real_T qorient_d, real_T *b_qorient_a,
    real_T *b_qorient_b, real_T *b_qorient_c, real_T *b_qorient_d);
  void IMU_Fusio_quaternionBase_rotmat(real_T q_a, real_T q_b, real_T q_c,
    real_T q_d, real_T r[9]);
  void IMU_Fusion_mrdiv(const real_T A[72], const real_T B[36], real_T Y[72]);
  void IMU_Fusion_ahrsfilter_stepImpl(fusion_simulink_ahrsfilter_IM_T *obj,
    const real_T accelIn[3], const real_T gyroIn[3], const real_T magIn[3],
    real_T orientOut[4], real_T av[3]);
  void IMU_Fu_AHRSFilterBase_resetImpl(fusion_simulink_ahrsfilter_IM_T *obj);
};

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
//  '<Root>' : 'IMU_Fusion'
//  '<S1>'   : 'IMU_Fusion/Angular Velocity Conversion'
//  '<S2>'   : 'IMU_Fusion/Quaternions to Rotation Angles'
//  '<S3>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Angle Calculation'
//  '<S4>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S5>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S6>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S7>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S8>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S9>'   : 'IMU_Fusion/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S10>'  : 'IMU_Fusion/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'

#endif                                 // RTW_HEADER_IMU_Fusion_h_

//
// File trailer for generated code.
//
// [EOF]
//
